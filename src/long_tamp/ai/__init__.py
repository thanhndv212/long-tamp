"""One gateway to AI models (ADR-0006): any model behind an Anthropic- or
OpenAI-compatible API, through one call shape, configuration and error set.

    from long_tamp.ai import configure, make_client

    configure()                      # $LONG_TAMP_AI_ENV, localhost in containers
    client = make_client("openai:my-model")
    answer = client.complete_json(system, user, schema, role="goal")

See ``config`` (model names, env files) and ``client`` (the two adapters).
"""

from .client import (
    AIAuthError,
    AIBillingError,
    AIConnectionError,
    AIError,
    AIOutputError,
    AIRateLimitError,
    AIRefusalError,
    AIRequestError,
    AnthropicClient,
    CallRecord,
    ModelClient,
    OpenAIClient,
    make_client,
)
from .config import (
    DEFAULT_MODEL,
    ModelSpec,
    configure,
    host_endpoint,
    load_env_file,
    parse_model,
)

__all__ = [
    "AIAuthError",
    "AIBillingError",
    "AIConnectionError",
    "AIError",
    "AIOutputError",
    "AIRateLimitError",
    "AIRefusalError",
    "AIRequestError",
    "AnthropicClient",
    "CallRecord",
    "DEFAULT_MODEL",
    "ModelClient",
    "ModelSpec",
    "OpenAIClient",
    "configure",
    "host_endpoint",
    "load_env_file",
    "make_client",
    "parse_model",
]
