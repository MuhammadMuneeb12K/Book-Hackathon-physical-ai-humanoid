# Security & Authentication

- BetterAuth must follow official session management best practices
- Secrets must be stored in environment variables only
- Rate limiting required on all public endpoints
- All user input must be validated and sanitized
- HTTPS enforcement mandatory in production
- XSS protection required for all user-generated content