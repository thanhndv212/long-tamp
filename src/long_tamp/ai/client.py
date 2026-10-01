"""One call shape for any model: a JSON answer to a system and a user message.

``make_client("openai:gpt-x")`` returns a ``ModelClient`` whose
``complete_json(system, user, schema)`` returns the parsed JSON answer, by
either API:

- **Anthropic Messages** (``AnthropicClient``): structured output
  (``output_config.format``). Claude on Anthropic's own endpoint also gets the
  server-side refusal fallback and an effort level; other models and gateways
  get a plain request.
- **OpenAI Chat Completions** (``OpenAIClient``): ``response_format`` with
  the JSON schema.

Where an endpoint rejects the JSON schema (400/422), the client asks again
once with the schema in the instructions and parses the JSON from the text,
and keeps doing so for that client.

Errors are typed (``AIError`` and its subclasses), whichever SDK raised them.
Every call, successful or not, is reported to ``on_call`` as a ``CallRecord``
(role, model, tokens, latency, error): the mission writes them to its event
stream. Neither SDK is needed to import this module.
"""

from __future__ import annotations

import json
import re
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .config import ModelSpec, host_endpoint, parse_model

# -- errors ------------------------------------------------------------------


class AIError(RuntimeError):
    """A model call failed; subclasses say how."""


class AIAuthError(AIError):
    """The endpoint rejected the credentials (or they are missing)."""


class AIBillingError(AIError):
    """The account can't pay for the call (no credit, quota exhausted)."""


class AIRateLimitError(AIError):
    """Too many requests; retry later."""


class AIConnectionError(AIError):
    """The endpoint can't be reached (or timed out)."""


class AIRefusalError(AIError):
    """The model declined to answer."""


class AIOutputError(AIError):
    """The answer is unusable: cut off, empty, or not the JSON asked for."""


class AIRequestError(AIError):
    """The endpoint rejected the request for another reason."""


def _classify(error: Exception) -> AIError:
    """The SDK's error as an ``AIError`` (by class name and status: both
    SDKs name their errors alike)."""
    name = type(error).__name__
    status = getattr(error, "status_code", None)
    message = getattr(error, "message", None) or str(error) or name
    text = message.lower()
    if name in ("AuthenticationError", "PermissionDeniedError") or status in (401, 403):
        return AIAuthError(message)
    if status == 402 or any(w in text for w in ("credit balance", "billing", "quota")):
        return AIBillingError(message)
    if name == "RateLimitError" or status == 429:
        return AIRateLimitError(message)
    if name in ("APIConnectionError", "APITimeoutError"):
        return AIConnectionError(message)
    return AIRequestError(message)


def _is_sdk_error(error: Exception) -> bool:
    return type(error).__module__.split(".")[0] in ("anthropic", "openai")


# -- records -----------------------------------------------------------------


@dataclass
class CallRecord:
    """One model call, for the event stream and cost accounting."""

    role: str
    model: str
    seconds: float
    input_tokens: int | None = None
    output_tokens: int | None = None
    error: str | None = None
    #: Whether the answer came in plain-JSON mode (the schema was refused).
    json_mode: bool = False

    def metrics(self) -> dict[str, Any]:
        return {
            k: v
            for k, v in {
                "model": self.model,
                "seconds": round(self.seconds, 3),
                "input_tokens": self.input_tokens,
                "output_tokens": self.output_tokens,
                "json_mode": self.json_mode or None,
            }.items()
            if v is not None
        }


# -- clients -----------------------------------------------------------------


def _json_instruction(schema: dict[str, Any]) -> str:
    return (
        "\nAnswer with one JSON object, and nothing else, matching this JSON "
        f"schema:\n{json.dumps(schema)}"
    )


def _parse_json(text: str) -> Any:
    """The JSON in ``text`` (a bare object, or one inside a code fence)."""
    text = text.strip()
    fence = re.search(r"```(?:json)?\s*(.*?)```", text, re.S)
    if fence:
        text = fence.group(1).strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError as error:
        raise AIOutputError(f"the answer is not JSON: {text[:200]!r}") from error


@dataclass
class ModelClient:
    """A model behind one API. Subclasses implement ``_call``."""

    spec: ModelSpec
    base_url: str | None = None
    api_key: str | None = None
    max_tokens: int = 16000
    timeout: float = 120.0
    max_retries: int = 2
    #: Receives every call's record (successful or not).
    on_call: Callable[[CallRecord], None] | None = None
    #: The SDK client; built on first use unless given.
    sdk: Any = None
    records: list[CallRecord] = field(default_factory=list)
    _json_schema: bool = True

    def complete_json(
        self,
        system: str,
        user: str,
        schema: dict[str, Any],
        role: str = "model",
    ) -> Any:
        """The model's JSON answer to ``user`` under ``system``, matching
        ``schema`` (checked by the endpoint when it supports schemas; the
        caller checks the content either way)."""
        t0 = time.monotonic()
        record = CallRecord(role=role, model=str(self.spec), seconds=0.0)
        try:
            try:
                text, usage = self._call(system, user, schema, self._json_schema)
            except Exception as error:
                status = getattr(error, "status_code", None)
                if not (
                    self._json_schema and _is_sdk_error(error) and status in (400, 422)
                ):
                    raise
                # The endpoint refused the schema: ask in plain JSON mode.
                self._json_schema = False
                text, usage = self._call(system, user, schema, False)
            record.json_mode = not self._json_schema
            record.input_tokens, record.output_tokens = usage
            return _parse_json(text)
        except AIError as error:
            record.error = f"{type(error).__name__}: {error}"
            raise
        except Exception as error:
            if not _is_sdk_error(error):
                raise
            mapped = _classify(error)
            record.error = f"{type(mapped).__name__}: {mapped}"
            raise mapped from error
        finally:
            record.seconds = time.monotonic() - t0
            self.records.append(record)
            if self.on_call is not None:
                self.on_call(record)

    def _call(
        self, system: str, user: str, schema: dict[str, Any], with_schema: bool
    ) -> tuple[str, tuple[int | None, int | None]]:
        raise NotImplementedError


@dataclass
class AnthropicClient(ModelClient):
    """The Anthropic Messages API (``pip install long-tamp[ai-anthropic]``)."""

    #: Effort for Claude on Anthropic's endpoint (Claude Opus 5.5's default is
    #: medium; set it explicitly).
    effort: str = "medium"

    def _sdk(self) -> Any:
        if self.sdk is None:
            try:
                import anthropic
            except ImportError as error:  # pragma: no cover - depends on the extra
                raise ImportError(
                    "the Anthropic API needs its SDK: pip install long-tamp[ai-anthropic]"
                ) from error
            self.sdk = anthropic.Anthropic(
                base_url=self.base_url,
                api_key=self.api_key,
                timeout=self.timeout,
                max_retries=self.max_retries,
            )
        return self.sdk

    def _first_party_claude(self) -> bool:
        """Claude on Anthropic's own endpoint: the one that takes the
        refusal fallback and effort (third-party gateways may not)."""
        url = self.base_url or ""
        if not url:
            import os

            url = os.environ.get("ANTHROPIC_BASE_URL", "")
        return self.spec.model.startswith("claude-") and (
            not url or "api.anthropic.com" in url
        )

    def _call(self, system, user, schema, with_schema):
        sdk = self._sdk()
        kwargs: dict[str, Any] = {
            "model": self.spec.model,
            "max_tokens": self.max_tokens,
            "system": system if with_schema else system + _json_instruction(schema),
            "messages": [{"role": "user", "content": user}],
        }
        output_config: dict[str, Any] = {}
        if with_schema:
            output_config["format"] = {"type": "json_schema", "schema": schema}
        if self._first_party_claude():
            output_config["effort"] = self.effort
            kwargs["betas"] = ["server-side-fallback-2026-07-01"]
            kwargs["fallbacks"] = "default"
        if output_config:
            kwargs["output_config"] = output_config
        create = sdk.beta.messages.create if "betas" in kwargs else sdk.messages.create
        response = create(**kwargs)
        if response.stop_reason == "refusal":
            details = getattr(response, "stop_details", None)
            category = getattr(details, "category", None) if details else None
            raise AIRefusalError(
                "the model declined" + (f" ({category})" if category else "")
            )
        if response.stop_reason == "max_tokens":
            raise AIOutputError("the answer was cut off (max_tokens)")
        text = "".join(b.text for b in response.content if b.type == "text")
        usage = getattr(response, "usage", None)
        return text, (
            getattr(usage, "input_tokens", None),
            getattr(usage, "output_tokens", None),
        )


@dataclass
class OpenAIClient(ModelClient):
    """An OpenAI-compatible Chat Completions endpoint
    (``pip install long-tamp[ai-openai]``)."""

    def _sdk(self) -> Any:
        if self.sdk is None:
            try:
                import openai
            except ImportError as error:  # pragma: no cover - depends on the extra
                raise ImportError(
                    "OpenAI-compatible endpoints need the OpenAI SDK: "
                    "pip install long-tamp[ai-openai]"
                ) from error
            self.sdk = openai.OpenAI(
                base_url=self.base_url,
                api_key=self.api_key,
                timeout=self.timeout,
                max_retries=self.max_retries,
            )
        return self.sdk

    def _call(self, system, user, schema, with_schema):
        if with_schema:
            response_format = {
                "type": "json_schema",
                "json_schema": {"name": "answer", "schema": schema, "strict": True},
            }
        else:
            response_format = {"type": "json_object"}
            system = system + _json_instruction(schema)
        response = self._sdk().chat.completions.create(
            model=self.spec.model,
            max_completion_tokens=self.max_tokens,
            messages=[
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            response_format=response_format,
        )
        choice = response.choices[0]
        if choice.finish_reason == "length":
            raise AIOutputError("the answer was cut off (max_tokens)")
        refusal = getattr(choice.message, "refusal", None)
        if refusal:
            raise AIRefusalError(f"the model declined: {refusal}")
        usage = getattr(response, "usage", None)
        return choice.message.content or "", (
            getattr(usage, "prompt_tokens", None),
            getattr(usage, "completion_tokens", None),
        )


def make_client(model: str | ModelSpec | None = None, **kwargs: Any) -> ModelClient:
    """A client for ``model`` (``"<api>:<model>"``; default: ``default_model()``,
    ``$LONG_TAMP_GOAL_MODEL`` or ``anthropic:claude-opus-5-5``). ``base_url`` defaults to the API's
    environment variable, with ``localhost`` mapped inside a container."""
    import os

    spec = model if isinstance(model, ModelSpec) else parse_model(model)
    variable = "ANTHROPIC_BASE_URL" if spec.api == "anthropic" else "OPENAI_BASE_URL"
    kwargs.setdefault("base_url", host_endpoint(os.environ.get(variable)) or None)
    cls = AnthropicClient if spec.api == "anthropic" else OpenAIClient
    return cls(spec=spec, **kwargs)
