# Tech Stack

This document outlines the technology stack for the Intelligent Documentation Assistant.

## Backend

-   **Framework**: FastAPI (Python)
-   **Language**: Python 3.11+
-   **Asynchronous Task Queue**: Celery with RabbitMQ as a message broker
-   **Database ORM**: SQLAlchemy

## Frontend

-   **Framework**: React.js
-   **Language**: TypeScript
-   **State Management**: Redux Toolkit
-   **UI Library**: Material-UI (MUI)

## Database

-   **Relational Database**: PostgreSQL
-   **Vector Database**: Pinecone

## Machine Learning / RAG

-   **Embedding Model**: Sentence-Transformers (e.g., `all-MiniLM-L6-v2`)
-   **LLM for Generation**: OpenAI GPT-3.5-turbo or a similar large language model
-   **PDF Parsing**: PyMuPDF

## DevOps

-   **Containerization**: Docker
-   **Orchestration**: Docker Compose for local development, Kubernetes for production
-   **CI/CD**: GitHub Actions
-   **Hosting**: AWS (e.g., EC2 for backend, S3 for static assets, RDS for PostgreSQL)
