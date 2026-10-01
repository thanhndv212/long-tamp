# 0006. AI model integration: one gateway, typed roles, gated tools

- Status: Proposed
- Date: 2026-10-01

## Context

#22 lets a model write a mission's goal. It has two writers (Claude through the Anthropic
API, any model behind an OpenAI-compatible endpoint), each with its own client code,
credentials handling and error mapping. Most model providers and gateways speak one or both of
those two APIs (Anthropic, OpenAI, Azure OpenAI, vLLM, Ollama, OpenRouter, LiteLLM, and
others), so one gateway with two adapters covers nearly every model.

The next uses go further than a goal:
- an operator who builds, adjusts and refines a plan and its execution in conversation;
- missions that run from an instruction to the end without a human in the loop.

ADR-0001 fixed the rule these must keep: a model writes the problem, never the plan, and every
input passes the same validation gate.

## Decision

1. **One gateway, `long_tamp.ai`.**
   - A `ModelClient` with two adapters: Anthropic Messages and OpenAI Chat Completions.
   - A model is named `<api>:<model>` (`anthropic:claude-opus-5-5`, `openai:gpt-…`), with its
     endpoint and key from environment variables or an env file. The file is parsed like a
     shell would parse it; inside a container, `localhost` maps to the Docker host.
   - It provides structured output (a JSON schema, falling back to JSON mode), retries,
     timeouts, and typed errors (auth, billing, rate limit, refusal, bad output).
   - Every call is recorded in the mission's event stream: role, model, prompt, response,
     tokens, latency.
   - SDKs stay optional extras. No provider is required to install or test long_tamp.
2. **Models act through typed roles.** Each role has an input contract, an output contract,
   a deterministic checker and a fallback that needs no model:
   - **Goal writer** (#22): an instruction becomes a goal, checked against the vocabulary and
     by the task planner.
   - **Grounder:** which scene objects an instruction refers to. Text first, vision later.
   - **Plan reviewer:** proposes constraints or blocked bindings, never steps; the planner
     validates them.
   - **Execution supervisor:** reads the event stream and picks one of a fixed set of actions
     (retry, replan, relax the goal, abort, escalate).

   A role's output that fails its checker goes back to the model with the reasons, for a
   bounded number of rounds. After that, the fallback applies.
3. **Interaction is a set of gated tools over a mission session.** An operator talks to a
   model that calls typed tools: set or adjust the goal, add a constraint, block a binding,
   plan, run, pause, resume, inspect events, explain a failure.
   - Each tool goes through the same checks as the Python API. Nothing a model does bypasses
     validation.
   - The front ends are a terminal chat first, then the web mission viewer (#24).
4. **Autonomy is the same loop, run by the supervisor role, under explicit limits.**
   - The loop: instruction, goal, plan, execute, monitor, repair.
   - Deterministic repair (refiner facts, planner replans) always goes first. The model only
     makes goal-level decisions.
   - The limits are rounds, wall-clock, model cost, and an allowlist of actions. When a limit
     is hit, or the checks reject every option, the mission stops and escalates to a human
     with a report, instead of guessing.

## Consequences

- Adding a provider means configuration, not code, when it speaks either API.
- Model behavior is observable and replayable from the event stream (recorded calls).
- Safety rests on the checkers, not on the model: an autonomous run can be wrong only in ways
  the planner, refiner and executor already allow.
- Each role needs its own checker and test. Fake clients keep the test suite free of API
  calls; live runs are smoke missions, recorded under `script/screw_assembly/results/`.
- PDDLStream (#23) leaves M6 for the backlog: this work takes its place.
