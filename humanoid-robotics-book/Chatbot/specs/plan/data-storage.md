# Data Storage

This document describes the data models and storage strategy for the Intelligent Documentation Assistant.

## Relational Database (PostgreSQL)

The PostgreSQL database will be the source of truth for all structured data. The following tables will be created:

### `users`

| Column | Type | Constraints |
|---|---|---|
| `id` | UUID | Primary Key |
| `name` | VARCHAR(255) | Not Null |
| `email` | VARCHAR(255) | Unique, Not Null |
| `password_hash` | VARCHAR(255) | Nullable (for social auth) |
| `auth_provider` | VARCHAR(50) | Not Null |
| `role` | VARCHAR(50) | Not Null, Default: 'user' |
| `created_at` | TIMESTAMP | Not Null, Default: CURRENT_TIMESTAMP |

### `conversations`

| Column | Type | Constraints |
|---|---|---|
| `id` | UUID | Primary Key |
| `user_id` | UUID | Foreign Key to `users.id` |
| `title` | VARCHAR(255) | Not Null |
| `created_at` | TIMESTAMP | Not Null, Default: CURRENT_TIMESTAMP |

### `messages`

| Column | Type | Constraints |
|---|---|---|
| `id` | UUID | Primary Key |
| `conversation_id` | UUID | Foreign Key to `conversations.id` |
| `sender` | VARCHAR(50) | Not Null ('user' or 'assistant') |
| `content` | TEXT | Not Null |
| `created_at` | TIMESTAMP | Not Null, Default: CURRENT_TIMESTAMP |

### `bookmarks`

| Column | Type | Constraints |
|---|---|---|
| `id` | UUID | Primary Key |
| `user_id` | UUID | Foreign Key to `users.id` |
| `content_reference` | VARCHAR(255) | Not Null (e.g., section ID) |
| `created_at` | TIMESTAMP | Not Null, Default: CURRENT_TIMESTAMP |

## Vector Database (Pinecone)

The Pinecone vector database will store the embeddings of the book content. Each record will contain:

-   **Vector ID**: A unique identifier for the vector.
-   **Embedding**: The vector representation of the text chunk.
-   **Metadata**: The original text chunk and its source (e.g., page number, section).
