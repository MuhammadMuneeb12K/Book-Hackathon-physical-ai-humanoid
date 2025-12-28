# Authentication

This document describes the authentication and authorization mechanism for the Intelligent Documentation Assistant.

## Authentication Flow

1.  **User Registration**:
    -   The user provides their name, email, and password.
    -   The **Authentication Service** validates the input, hashes the password using a strong hashing algorithm (e.g., bcrypt), and stores the user information in the `users` table in the PostgreSQL database.

2.  **User Login**:
    -   The user provides their email and password.
    -   The **Authentication Service** verifies the credentials against the stored hash.
    -   Upon successful verification, the service generates a JSON Web Token (JWT) containing the user's ID and role.

3.  **Session Management**:
    -   The JWT is sent to the frontend and stored securely (e.g., in an HttpOnly cookie).
    -   For subsequent requests, the JWT is included in the `Authorization` header.
    -   The **Backend API Service** validates the JWT and extracts the user's information.

4.  **Token Refresh**:
    -   The JWT will have a limited lifespan (e.g., 15 minutes).
    -   A refresh token with a longer lifespan (e.g., 7 days) will also be issued upon login and stored securely.
    -   When the access token expires, the frontend can use the refresh token to request a new access token from the **Authentication Service**.

## Social Authentication

-   The system will support social authentication via Google and GitHub.
-   The flow will use the OAuth 2.0 protocol.
-   Upon successful authentication with the social provider, a JWT will be issued, and a new user account will be created if one doesn't already exist.

## Role-Based Access Control (RBAC)

-   The system will have at least two roles: `user` and `admin`.
-   The user's role will be included in the JWT.
-   The **Backend API Service** will use this role to enforce access control on different API endpoints.
