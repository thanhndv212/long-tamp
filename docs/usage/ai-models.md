# AI models

long-tamp uses AI models through one gateway, `long_tamp.ai` (ADR-0006). A model never writes
the plan: it writes problems (today, a mission's goal from an instruction), and long-tamp
checks everything it writes before using it.

## Naming a model

A model is named `<api>:<model>`: the API it is reached through, then the model id the
endpoint knows it by.

| API | Speaks | Examples |
|---|---|---|
| `anthropic` | Anthropic Messages | `anthropic:claude-opus-5-5` (the default), any gateway serving the Messages API |
| `openai` | OpenAI Chat Completions | OpenAI, Azure OpenAI, vLLM, Ollama, OpenRouter, LiteLLM, local gateways |

Most providers and gateways speak one of the two, often both. Adding one is configuration,
not code.

Install the SDK for the API you use:

```bash
pip install "long-tamp[ai-anthropic]"   # Anthropic Messages
pip install "long-tamp[ai-openai]"      # OpenAI-compatible endpoints
pip install "long-tamp[ai]"             # both
```

## Endpoints and keys

Each SDK reads its own variables:

| API | Key | Endpoint (optional) |
|---|---|---|
| `anthropic` | `ANTHROPIC_API_KEY` (or an `ant auth login` profile) | `ANTHROPIC_BASE_URL` |
| `openai` | `OPENAI_API_KEY` | `OPENAI_BASE_URL` |

Keep them in an env file, and point long-tamp at it with `--ai-env FILE` or
`LONG_TAMP_AI_ENV`:

```bash
# ~/devel/hpp/.anthropic/env (folder mode 700, so the file stays private)
export OPENAI_API_KEY="..."                   # comments and quotes are fine
export OPENAI_BASE_URL="http://localhost:20128/v1"
```

The file is read the way a shell reads it (`export`, quotes, trailing comments). Variables
already set in the environment win over the file.

### In a container

- **`localhost`:** an endpoint on `localhost` or `127.0.0.1` means the host's, so inside a
  Docker or Podman container it is rewritten to `host.docker.internal`. Set
  `LONG_TAMP_AI_MAP_LOCALHOST=0` to turn that off.
- **File permissions:** a file mounted from the host must be readable by the container's
  user. With macOS + OrbStack, a `600` file isn't readable from the container. Use mode `644`
  inside a folder of mode `700`: other users of the host can't enter the folder.
- **No writable home** (the HPP container's home is owned by root): set
  `ANTHROPIC_CONFIG_DIR` to a writable folder before `ant auth login`.

## Use it from Python

```python
from long_tamp.ai import configure, make_client
from long_tamp.tasks.task_planning.language import ModelGoalWriter, goal_from_instruction

configure("~/devel/hpp/.anthropic/env")      # or $LONG_TAMP_AI_ENV
client = make_client("openai:my-model", on_call=print)
goal = goal_from_instruction("assemble part 2", ModelGoalWriter(client), vocabulary, reachable)
```

`client.complete_json(system, user, schema, role=...)` returns the model's JSON answer:

- **Structured output:** the endpoint gets the JSON schema when it takes one. When it
  refuses the schema (400/422), the client asks again in plain JSON mode and keeps doing so
  for that client.
- **Claude on Anthropic's own endpoint** also gets an effort level and the server-side
  refusal fallback. Other models and gateways get a plain request.

## Errors

Failures are typed, whichever SDK raised them:

| Error | Meaning |
|---|---|
| `AIAuthError` | missing or rejected credentials |
| `AIBillingError` | no credit, or quota exhausted |
| `AIRateLimitError` | too many requests |
| `AIConnectionError` | endpoint unreachable, or timed out |
| `AIRefusalError` | the model declined |
| `AIOutputError` | the answer was cut off, empty, or not JSON |
| `AIRequestError` | anything else the endpoint rejected |

## Recorded calls

Every call, successful or not, produces a `CallRecord`: role, model, seconds, tokens, error.
`task_screw_assembly.py` writes them to the mission's `events.jsonl` as `model` events:

```json
{"role": "model", "name": "goal", "status": "SUCCESS",
 "metrics": {"model": "openai:cx/gpt-6.1-sol", "input_tokens": 620, "output_tokens": 386, "seconds": 23.8}}
```

## Roles

Models act through roles (`long_tamp.ai.roles`). In each role, the model proposes and long-tamp
checks:

| Role | Writes | Checked by | Fallback |
|---|---|---|---|
| grounder | the objects an instruction refers to | known objects | name matching |
| goal writer | the goal (final-state literals) | vocabulary, then the task planner reaches it | none: the mission stops |
| plan reviewer | constraints: capability + parameters to avoid | known names, then the planner still reaches the goal | no constraint |

A rejected proposal goes back to the model with the checker's reasons, for a bounded number
of rounds. No role writes plan steps. A new role is a `ModelRole` (system prompt, JSON
schema, how a request becomes a prompt, how an answer becomes a value), a checker and a
fallback, run with `refine`.

## The screw-assembly mission

```bash
python3 task_screw_assembly.py --backend mujoco \
  --ai-env ~/devel/hpp/.anthropic/env --goal-model openai:my-model \
  --instruction "assemble part 1, leave part 2 alone, and rack the driver when done"
```

The grounder finds what the instruction refers to and the goal writer writes the goal.
long_tamp checks its syntax, predicates and objects, then whether the task planner can reach
it from the observed state. The plan reviewer turns what the instruction rules out into
constraints. The planner plans the goal under those constraints, and the mission executes the
plan.
