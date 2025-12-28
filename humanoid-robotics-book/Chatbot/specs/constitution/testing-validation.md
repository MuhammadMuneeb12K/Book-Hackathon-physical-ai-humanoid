# Testing & Validation Standards

- Unit tests required for critical business logic (target ≥80% coverage)
- Integration tests required for API endpoints and RAG pipelines
- Type safety enforced; `any` only allowed for untyped external libraries
- Manual testing required in light and dark modes before commits
- Edge cases must cover loading, empty, and error states