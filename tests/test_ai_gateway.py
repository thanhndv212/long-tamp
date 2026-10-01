"""The AI gateway (#86): one call shape for Anthropic- and OpenAI-compatible
APIs. No API calls: the SDKs are faked."""

import json
from types import SimpleNamespace

import pytest

from long_tamp.ai import (
    AIAuthError,
    AIBillingError,
    AIConnectionError,
    AIOutputError,
    AIRateLimitError,
    AIRefusalError,
    AnthropicClient,
    OpenAIClient,
    configure,
    host_endpoint,
    load_env_file,
    make_client,
    parse_model,
)
from long_tamp.ai import config as ai_config

SCHEMA = {
    "type": "object",
    "properties": {"x": {"type": "integer"}},
    "required": ["x"],
    "additionalProperties": False,
}

# -- configuration -------------------------------------------------------------


def test_models_are_named_api_colon_model():
    assert (
        parse_model("openai:llama3.1").api,
        parse_model("openai:llama3.1").model,
    ) == (
        "openai",
        "llama3.1",
    )
    assert str(parse_model("anthropic:claude-opus-5-5")) == "anthropic:claude-opus-5-5"
    assert parse_model("claude-sonnet-5-5").api == "anthropic"  # bare Claude id
    assert str(parse_model(None)) == "anthropic:claude-opus-5-5"
    # a colon inside the model id stays in the id
    assert parse_model("openai:cx/gpt:latest").model == "cx/gpt:latest"
    with pytest.raises(ValueError, match="<api>:<model>"):
        parse_model("gpt-x")
    with pytest.raises(ValueError):
        parse_model("gemini:x")


def test_env_files_are_read_like_a_shell_reads_them(tmp_path, monkeypatch):
    for name in ("A_KEY", "A_URL", "A_MODEL", "A_KEPT"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("A_KEPT", "from-env")
    env = tmp_path / "ai.env"
    env.write_text(
        "# credentials\n"
        'export A_KEY="sk-abc 123"   # the key\n'
        "A_URL='http://localhost:20128/v1'\n"
        "export A_MODEL=cx/gpt-6.1   # trailing comment\n"
        "A_KEPT=from-file\n"
        "not a variable line\n"
    )
    names = load_env_file(env)
    assert names == ["A_KEY", "A_URL", "A_MODEL", "A_KEPT"]
    import os

    assert os.environ["A_KEY"] == "sk-abc 123"
    assert os.environ["A_URL"] == "http://localhost:20128/v1"
    assert os.environ["A_MODEL"] == "cx/gpt-6.1"
    assert os.environ["A_KEPT"] == "from-env"  # the environment wins


def test_localhost_means_the_host_inside_a_container(monkeypatch):
    monkeypatch.delenv("LONG_TAMP_AI_MAP_LOCALHOST", raising=False)
    monkeypatch.setattr(ai_config, "in_container", lambda: True)
    assert (
        host_endpoint("http://localhost:20128/v1")
        == "http://host.docker.internal:20128/v1"
    )
    assert host_endpoint("http://127.0.0.1/v1") == "http://host.docker.internal/v1"
    assert host_endpoint("https://api.example.com/v1") == "https://api.example.com/v1"
    monkeypatch.setenv("LONG_TAMP_AI_MAP_LOCALHOST", "0")
    assert host_endpoint("http://localhost:1/v1") == "http://localhost:1/v1"
    monkeypatch.delenv("LONG_TAMP_AI_MAP_LOCALHOST")
    monkeypatch.setattr(ai_config, "in_container", lambda: False)
    assert host_endpoint("http://localhost:1/v1") == "http://localhost:1/v1"


def test_configure_loads_the_env_file_and_maps_endpoints(tmp_path, monkeypatch):
    monkeypatch.delenv("OPENAI_BASE_URL", raising=False)
    monkeypatch.setattr(ai_config, "in_container", lambda: True)
    env = tmp_path / "ai.env"
    env.write_text("export OPENAI_BASE_URL=http://localhost:9/v1\n")
    monkeypatch.setenv("LONG_TAMP_AI_ENV", str(env))
    assert configure() == ["OPENAI_BASE_URL"]
    import os

    assert os.environ["OPENAI_BASE_URL"] == "http://host.docker.internal:9/v1"


def test_make_client_picks_the_adapter_and_endpoint(monkeypatch):
    monkeypatch.setenv("OPENAI_BASE_URL", "https://gw.example/v1")
    client = make_client("openai:m")
    assert (
        isinstance(client, OpenAIClient) and client.base_url == "https://gw.example/v1"
    )
    monkeypatch.delenv("ANTHROPIC_BASE_URL", raising=False)
    assert isinstance(make_client(), AnthropicClient)


# -- fakes ---------------------------------------------------------------------


def SDKError(name, status=None, message="boom"):
    """An error like the SDK's: its module and class name are what count."""
    cls = type(name, (Exception,), {"__module__": "openai._exceptions"})
    error = cls(message)
    error.status_code, error.message = status, message
    return error


class Replay:
    """A create() that records its calls and replays answers (or raises)."""

    def __init__(self, *answers):
        self.answers, self.calls = list(answers), []

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        answer = self.answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer


def anthropic_sdk(*answers):
    create = Replay(*answers)
    sdk = SimpleNamespace(
        messages=SimpleNamespace(create=create),
        beta=SimpleNamespace(messages=SimpleNamespace(create=create)),
    )
    return sdk, create


def message(text, stop_reason="end_turn", tokens=(10, 5)):
    return SimpleNamespace(
        stop_reason=stop_reason,
        stop_details=None,
        content=[SimpleNamespace(type="text", text=text)],
        usage=SimpleNamespace(input_tokens=tokens[0], output_tokens=tokens[1]),
    )


def openai_sdk(*answers):
    create = Replay(*answers)
    return (
        SimpleNamespace(
            chat=SimpleNamespace(completions=SimpleNamespace(create=create))
        ),
        create,
    )


def completion(text, finish_reason="stop", refusal=None, tokens=(10, 5)):
    msg = SimpleNamespace(content=text, refusal=refusal)
    return SimpleNamespace(
        choices=[SimpleNamespace(message=msg, finish_reason=finish_reason)],
        usage=SimpleNamespace(prompt_tokens=tokens[0], completion_tokens=tokens[1]),
    )


# -- Anthropic adapter ---------------------------------------------------------


def test_claude_on_anthropics_endpoint_gets_structured_output_and_fallbacks(
    monkeypatch,
):
    monkeypatch.delenv("ANTHROPIC_BASE_URL", raising=False)
    sdk, create = anthropic_sdk(message('{"x": 1}'))
    records = []
    client = make_client("anthropic:claude-opus-5-5", sdk=sdk, on_call=records.append)
    assert client.complete_json("sys", "user", SCHEMA, role="goal") == {"x": 1}
    (call,) = create.calls
    assert call["output_config"]["format"] == {"type": "json_schema", "schema": SCHEMA}
    assert call["output_config"]["effort"] == "medium"
    assert call["fallbacks"] == "default" and call["betas"]
    (record,) = records
    assert (record.role, record.input_tokens, record.output_tokens) == ("goal", 10, 5)
    assert (
        record.error is None
        and record.metrics()["model"] == "anthropic:claude-opus-5-5"
    )


def test_other_models_on_an_anthropic_gateway_get_a_plain_request():
    sdk, create = anthropic_sdk(message('{"x": 2}'))
    client = AnthropicClient(
        spec=parse_model("anthropic:kimi-k3"), base_url="http://gw/anthropic", sdk=sdk
    )
    assert client.complete_json("sys", "user", SCHEMA) == {"x": 2}
    (call,) = create.calls
    assert "betas" not in call and "fallbacks" not in call
    assert "effort" not in call["output_config"]


def test_a_refused_schema_falls_back_to_json_in_the_instructions():
    sdk, create = anthropic_sdk(
        SDKError("BadRequestError", 400, "output_config not supported"),
        message('Here it is:\n```json\n{"x": 3}\n```'),
    )
    client = AnthropicClient(
        spec=parse_model("anthropic:m"), base_url="http://gw", sdk=sdk
    )
    assert client.complete_json("sys", "user", SCHEMA) == {"x": 3}
    second = create.calls[1]
    assert "output_config" not in second and "JSON schema" in second["system"]
    assert client.records[-1].json_mode
    # and keeps doing so
    sdk.messages.create.answers.append(message('{"x": 4}'))
    client.complete_json("sys", "user", SCHEMA)
    assert "output_config" not in create.calls[2]


def test_anthropic_refusal_and_truncation_are_typed():
    sdk, _ = anthropic_sdk(
        message("", stop_reason="refusal"), message("{", stop_reason="max_tokens")
    )
    client = AnthropicClient(
        spec=parse_model("anthropic:m"), base_url="http://gw", sdk=sdk
    )
    with pytest.raises(AIRefusalError):
        client.complete_json("s", "u", SCHEMA)
    with pytest.raises(AIOutputError, match="cut off"):
        client.complete_json("s", "u", SCHEMA)
    assert all(r.error for r in client.records)


# -- OpenAI adapter --------------------------------------------------------------


def test_openai_compatible_endpoints_get_a_json_schema():
    sdk, create = openai_sdk(completion(json.dumps({"x": 5})))
    records = []
    client = make_client(
        "openai:m", sdk=sdk, on_call=records.append, base_url="http://gw/v1"
    )
    assert client.complete_json("sys", "user", SCHEMA, role="review") == {"x": 5}
    (call,) = create.calls
    assert call["response_format"]["type"] == "json_schema"
    assert call["messages"][0] == {"role": "system", "content": "sys"}
    assert records[0].role == "review" and records[0].output_tokens == 5


def test_an_endpoint_without_schemas_gets_json_mode():
    sdk, create = openai_sdk(
        SDKError("BadRequestError", 400, "response_format json_schema unsupported"),
        completion('{"x": 6}'),
    )
    client = OpenAIClient(spec=parse_model("openai:m"), sdk=sdk)
    assert client.complete_json("sys", "user", SCHEMA) == {"x": 6}
    assert create.calls[1]["response_format"] == {"type": "json_object"}
    assert "JSON schema" in create.calls[1]["messages"][0]["content"]


@pytest.mark.parametrize(
    "error, expected",
    [
        (SDKError("AuthenticationError", 401), AIAuthError),
        (
            SDKError("BadRequestError", 400, "Your credit balance is too low"),
            AIBillingError,
        ),
        (SDKError("RateLimitError", 429), AIRateLimitError),
        (SDKError("APIConnectionError", None, "Connection error."), AIConnectionError),
    ],
)
def test_sdk_errors_become_typed_ai_errors(error, expected):
    # 400s first retry in JSON mode; a second 400 is then classified
    answers = [error, error] if getattr(error, "status_code", None) == 400 else [error]
    sdk, _ = openai_sdk(*answers)
    records = []
    client = OpenAIClient(spec=parse_model("openai:m"), sdk=sdk, on_call=records.append)
    with pytest.raises(expected):
        client.complete_json("s", "u", SCHEMA)
    assert records and records[0].error.startswith(expected.__name__)


def test_bad_json_is_an_output_error():
    sdk, _ = openai_sdk(completion("not json at all"))
    with pytest.raises(AIOutputError, match="not JSON"):
        OpenAIClient(spec=parse_model("openai:m"), sdk=sdk).complete_json(
            "s", "u", SCHEMA
        )
