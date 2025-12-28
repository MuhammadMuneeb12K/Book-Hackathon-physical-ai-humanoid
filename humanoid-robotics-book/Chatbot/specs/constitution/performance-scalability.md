# Performance & Scalability

- RAG responses must complete within 3 seconds
- Vector search scope must be restricted using Qdrant filters
- Lazy loading required for routes and heavy components
- Embeddings and frequent queries must be cached
- Frequently queried database fields must be indexed
- Initial JavaScript bundle size must remain under 300KB