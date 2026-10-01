"""Which model, at which endpoint, with which key: one configuration for every provider.

A model is named ``<api>:<model>``: the API it is reached through, then the
model id the endpoint knows it by.

- ``anthropic:claude-opus-5-5``: the Anthropic Messages API (Anthropic, or any
  gateway speaking it);
- ``openai:gpt-…``, ``openai:llama3.1``: an OpenAI-compatible Chat Completions
  endpoint (OpenAI, Azure OpenAI, vLLM, Ollama, OpenRouter, LiteLLM, …).

Endpoints and keys come from the environment, as each SDK reads them
(``ANTHROPIC_API_KEY`` / ``ANTHROPIC_BASE_URL``, ``OPENAI_API_KEY`` /
``OPENAI_BASE_URL``), optionally loaded from an env file
(``load_env_file``; ``configure()`` loads ``LONG_TAMP_AI_ENV``, else
``~/.config/long-tamp/ai.env`` if it exists, see ``default_env_file``). The model used when none is
named is ``LONG_TAMP_GOAL_MODEL`` (it may be set in that file), else
``anthropic:claude-opus-5-5``. Inside a container, an endpoint
on ``localhost`` means the host's, so it is rewritten to
``host.docker.internal`` (``LONG_TAMP_AI_MAP_LOCALHOST=0`` turns that off).
"""

from __future__ import annotations

import os
import re
import shlex
from dataclasses import dataclass
from pathlib import Path

#: The APIs a model can be reached through.
APIS = ("anthropic", "openai")

#: The default model, when none is named.
DEFAULT_MODEL = "anthropic:claude-opus-5-5"

#: Env file loaded by ``configure()`` when set.
ENV_FILE_VARIABLE = "LONG_TAMP_AI_ENV"
#: Env file loaded by ``configure()`` when no other is named, if it exists
#: (under ``$XDG_CONFIG_HOME`` instead of ``~/.config`` when that is set).
DEFAULT_ENV_FILE = Path("~/.config/long-tamp/ai.env")


def default_env_file() -> Path:
    """``$XDG_CONFIG_HOME/long-tamp/ai.env``, else ``~/.config/long-tamp/ai.env``."""
    base = os.environ.get("XDG_CONFIG_HOME")
    if base:
        return Path(base) / "long-tamp" / "ai.env"
    return DEFAULT_ENV_FILE.expanduser()


#: The model used when none is named (``<api>:<model>``).
MODEL_VARIABLE = "LONG_TAMP_GOAL_MODEL"

_BASE_URL_VARIABLES = {"anthropic": "ANTHROPIC_BASE_URL", "openai": "OPENAI_BASE_URL"}


@dataclass(frozen=True)
class ModelSpec:
    """A model and the API it is reached through."""

    api: str
    model: str

    def __str__(self) -> str:
        return f"{self.api}:{self.model}"


def parse_model(name: str | None) -> ModelSpec:
    """``"openai:gpt-x"`` -> ModelSpec("openai", "gpt-x"). A bare Claude id
    (``claude-…``) means the Anthropic API; any other bare id is an error,
    since the API can't be guessed."""
    if not name:
        return default_model()
    name = name.strip()
    api, sep, model = name.partition(":")
    if sep and api in APIS and model:
        return ModelSpec(api, model)
    if not sep and name.startswith("claude-"):
        return ModelSpec("anthropic", name)
    raise ValueError(
        f"model {name!r}: name it <api>:<model>, with <api> one of {', '.join(APIS)} "
        "(e.g. anthropic:claude-opus-5-5, openai:gpt-…)"
    )


def default_model() -> ModelSpec:
    """The model named by ``LONG_TAMP_GOAL_MODEL``, else ``DEFAULT_MODEL``.

    A bare id there that isn't a Claude id is served by the one API whose
    endpoint is configured (``OPENAI_BASE_URL`` or ``ANTHROPIC_BASE_URL``):
    a local gateway's env file can name its model as the gateway knows it.
    """
    name = os.environ.get(MODEL_VARIABLE, "").strip()
    if not name:
        return parse_model(DEFAULT_MODEL)
    api, sep, model = name.partition(":")
    if (sep and api in APIS) or name.startswith("claude-"):
        return parse_model(name)
    configured = [a for a, v in _BASE_URL_VARIABLES.items() if os.environ.get(v)]
    if len(configured) == 1:
        return ModelSpec(configured[0], name)
    raise ValueError(
        f"{MODEL_VARIABLE}={name!r}: name it <api>:<model> (the API can't be "
        "guessed when no single endpoint is configured)"
    )


def load_env_file(path: str | os.PathLike, override: bool = False) -> list[str]:
    """Set the variables an env file assigns, read the way a shell reads it:
    ``export``, quotes and trailing comments are handled. Returns the names
    set (never the values). Existing variables win unless ``override``."""
    names = []
    for line in Path(path).read_text().splitlines():
        try:
            words = shlex.split(line, comments=True)
        except ValueError as error:
            raise ValueError(
                f"{path}: can't read line {line[:20]!r}…: {error}"
            ) from None
        if words and words[0] == "export":
            words = words[1:]
        if not words or "=" not in words[0]:
            continue
        key, value = words[0].split("=", 1)
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            continue
        if override or key not in os.environ:
            os.environ[key] = value
        names.append(key)
    return names


def in_container() -> bool:
    """Whether this process runs in a Docker/Podman container."""
    return Path("/.dockerenv").exists() or Path("/run/.containerenv").exists()


def host_endpoint(url: str | None) -> str | None:
    """``url`` as reachable from here: inside a container, an endpoint on the
    host's ``localhost`` becomes ``host.docker.internal``."""
    if not url or os.environ.get("LONG_TAMP_AI_MAP_LOCALHOST", "1") == "0":
        return url
    if not in_container():
        return url
    return re.sub(
        r"//(localhost|127\.0\.0\.1)(?=[:/]|$)", "//host.docker.internal", url
    )


def configure(env_file: str | os.PathLike | None = None) -> list[str]:
    """Load ``env_file`` (or ``$LONG_TAMP_AI_ENV``, or
    ``~/.config/long-tamp/ai.env`` if it exists), then map the endpoints'
    ``localhost`` inside a container. Returns the variable names loaded."""
    path = env_file or os.environ.get(ENV_FILE_VARIABLE)
    if not path and default_env_file().is_file():
        path = default_env_file()
    names = load_env_file(path) if path else []
    for variable in _BASE_URL_VARIABLES.values():
        if os.environ.get(variable):
            os.environ[variable] = host_endpoint(os.environ[variable])
    return names
