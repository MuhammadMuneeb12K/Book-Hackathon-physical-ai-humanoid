# Tasks: Intelligent Documentation Assistant

**Input**: Design documents from `/specs/004-intelligent-doc-assistant/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Foundation Setup

**Purpose**: Establish development environment and core infrastructure

- [X] T001 Create Next.js 14 project in `src/` with TypeScript and App Router
  * **Description**: This task involves scaffolding a new Next.js 14 project. The recommended command is `npx create-next-app@latest src --ts --app --tailwind --eslint --use-npm`. This command sets up the project with TypeScript, the App Router, Tailwind CSS, and ESLint, and uses npm as the package manager. The user should execute this command in the project root.
- [X] T002 Install dependencies (Tailwind, shadcn/ui, Prisma, BetterAuth) in `src/`
  * **Description**: After creating the Next.js project, install additional core dependencies. Tailwind CSS is typically set up during project creation. For `Prisma` and `BetterAuth`, execute `npm install prisma @prisma/client better-auth` within the `src/` directory. `shadcn/ui` components will be initialized and added in subsequent tasks using its CLI commands (e.g., `npx shadcn-ui@latest init`).
- [X] T003 Configure TypeScript with strict mode for `src/`
  * **Description**: Verify that the `tsconfig.json` file (usually in the project root) has `"strict": true` within the `compilerOptions`. The `create-next-app` command typically sets this by default. If not present, add `"strict": true`.
- [X] T004 Set up ESLint and Prettier with project rules for `src/`
  * **Description**: ESLint is typically configured during `create-next-app`. To set up Prettier, install it (`npm install --save-dev prettier eslint-config-prettier`). Create a `.prettierrc` file in the project root to define formatting rules (e.g., `{"semi": false, "singleQuote": true}`). Update `.eslintrc.json` to extend `eslint-config-prettier` to integrate Prettier with ESLint and prevent formatting conflicts. Ensure consistent formatting across the project.
- [X] T005 Create `.env.example` with all required variables in `src/`
  * **Description**: Create a file named `.env.example` in the project root. This file should list all environment variables required for the application to run, but with placeholder values. Examples include `DATABASE_URL`, `GEMINI_API_KEY`, `BETTER_AUTH_SECRET`, `NEXTAUTH_SECRET`, and `NEXTAUTH_URL`. This serves as a guide for developers to set up their own `.env.local` files, ensuring no sensitive information is hardcoded.
- [X] T006 Initialize Git repository with comprehensive `.gitignore`
  * **Description**: This task has already been completed. The Git repository is initialized, and a comprehensive `.gitignore` file has been created to prevent unwanted files from being committed. This ensures proper version control from the outset.
- [X] T007 Design PostgreSQL schema for `prisma/schema.prisma` (users, sessions, chat_history, bookmarks, user_progress)
  * **Description**: Design the data models for `User`, `Session`, `ChatHistory`, `Bookmark`, and `UserProgress` within the `prisma/schema.prisma` file. Define their fields, types, and relationships based on the "Data Storage Strategy" and "Key Entities" outlined in the plan and specification. This includes fields like `id`, `email`, `name`, `created_at` for `User`, and relevant fields for tracking chat, bookmarks, and progress.
- [X] T008 Create Prisma schema file `prisma/schema.prisma`
  * **Description**: Create the actual `prisma/schema.prisma` file based on the design from T007. This file will define the `datasource`, `generator`, and the Prisma models for `User`, `Session`, `ChatHistory`, `Bookmark`, and `UserProgress` with their respective fields and relations, aligning with the `PostgreSQL Schema` defined in the technical plan.
- [X] T009 Write initial database migration for `prisma/migrations/`
  * **Description**: Generate the initial database migration using `npx prisma migrate dev --name init`. This command will create a new migration file in `prisma/migrations/` based on the current `schema.prisma`, setting up the necessary tables in the PostgreSQL database.
- [X] T010 Set up local PostgreSQL instance using Docker Compose
  * **Description**: Create a `docker-compose.yml` file in the project root to define a PostgreSQL service. This file will specify the PostgreSQL image, environment variables (e.g., `POSTGRES_DB`, `POSTGRES_USER`, `POSTGRES_PASSWORD`), and port mappings. Run `docker compose up -d` to start the PostgreSQL container in the background.
- [X] T011 Run database migrations and verify connection
  * **Description**: Apply the database migrations using `npx prisma migrate deploy`. After deployment, verify the connection by running a simple Prisma query from your application code (e.g., `await prisma.$connect()`) or by starting the application and ensuring no database connection errors occur.
- [X] T012 Seed database with a test user via `prisma/seed.ts`
  * **Description**: Create a `prisma/seed.ts` file. This file will contain logic to use the Prisma client to insert initial data, including at least one test user, into the database. Add `"prisma": {"seed": "ts-node prisma/seed.ts"}` to your `package.json` scripts. Run the seeding script using `npx prisma db seed`.
- [X] T013 Set up local Qdrant instance using Docker
  * **Description**: Add a Qdrant service to the `docker-compose.yml` file. This service will use the Qdrant Docker image and expose the necessary ports (e.g., 6333 for HTTP API, 6334 for gRPC). Ensure volumes are configured for persistent storage. Run `docker compose up -d qdrant` to start the Qdrant container.
- [X] T014 Create Qdrant collection "humanoid_robotics_book" with vector size 768 (Gemini embedding dimensions)
  * **Description**: Using a Qdrant client (e.g., `@qdrant/qdrant-typescript`), create a new collection named "humanoid_robotics_book". The collection configuration should specify a `vector_size` of 768 (matching Gemini embedding output) and a `distance` metric of "Cosine" for similarity search. This can be done programmatically in an initialization script.
- [X] T015 Write utility script `src/utils/qdrant.ts` to test Qdrant connection
  * **Description**: Create a TypeScript utility file `src/utils/qdrant.ts`. This script will initialize the Qdrant client and include a function (e.g., `testConnection()`) that attempts to connect to the Qdrant instance and perform a basic operation, such as listing collections. This script will be used to verify that the Qdrant instance is reachable and correctly configured.
- [X] T016 Configure Qdrant HNSW index parameters for performance
  * **Description**: When creating or updating the "humanoid_robotics_book" collection in Qdrant, configure the HNSW index parameters. This involves setting `hnsw_config` properties such as `m` (e.g., 16-32), `ef_construct` (e.g., 100-200), and `full_scan_threshold`. These parameters are critical for optimizing search performance and recall. Refer to Qdrant documentation for optimal values based on data size and search requirements.
- [X] T017 Document Qdrant setup process in `docs/Qdrant_Setup.md`
  * **Description**: Create a Markdown file `docs/Qdrant_Setup.md`. This document should detail the entire process of setting up Qdrant locally using Docker, creating the "humanoid_robotics_book" collection, configuring its HNSW index parameters, and verifying the connection. Include command-line examples and configuration snippets.

---

## Phase 2: Authentication System (US2 - Secure User Authentication)

**Goal**: Implement secure user authentication with BetterAuth

### Tests for User Story 2

- [X] T018 [P] [US2] Unit test authentication utilities in `tests/auth/auth.test.ts`
  * **Description**: Create a test file `tests/auth/auth.test.ts`. Write unit tests for authentication-related utility functions, such as password hashing (e.g., `hashPassword`), token creation/verification, and input validation functions. These tests should cover various scenarios (valid/invalid inputs) and ensure the utilities function as expected.
- [X] T019 [P] [US2] Integration test user signup and login flow in `tests/integration/auth_flow.test.ts`
  * **Description**: Create an integration test file `tests/integration/auth_flow.test.ts`. This test should simulate the full user signup and login flow, interacting with the actual API routes and database. It should cover successful registration, successful login, incorrect credentials, and session validation, ensuring the entire authentication system works end-to-end.

### Implementation for User Story 2

- [X] T020 [US2] Install and configure BetterAuth with PostgreSQL adapter in `src/auth.ts`
  * **Description**: Install BetterAuth and its PostgreSQL adapter (`npm install better-auth @better-auth/adapter-prisma`). Create `src/auth.ts` to initialize BetterAuth, configuring it to use the Prisma adapter with the PostgreSQL database. This file will export the necessary BetterAuth handlers and configurations for the Next.js API routes.
- [X] T021 [US2] Set up authentication API routes (`/api/auth/[...all]`) in `src/pages/api/auth/[...all].ts`
  * **Description**: Create a file `src/pages/api/auth/[...all].ts`. This catch-all API route will handle all authentication requests (e.g., login, register, logout) as managed by BetterAuth. The file will export the BetterAuth handler.
- [X] T022 [US2] Configure session management (7-day sessions, sliding window, secure httpOnly cookies)
  * **Description**: Within the BetterAuth initialization in `src/auth.ts`, configure session management options. Set the session duration to 7 days, enable a sliding window (sessions extend on activity), and ensure session cookies are `secure` (sent only over HTTPS) and `httpOnly` (not accessible via JavaScript) for robust security.
- [X] T023 [US2] Implement email/password authentication using BetterAuth
  * **Description**: Configure BetterAuth in `src/auth.ts` to support email/password authentication. This involves defining the registration process (e.g., hashing passwords using `bcrypt`) and the login verification logic. Ensure that password hashing and comparison adhere to security best practices as outlined in the project constitution.
- [X] T024 [US2] Add OAuth providers (Google, GitHub) using BetterAuth
  * **Description**: Extend BetterAuth configuration in `src/auth.ts` to include OAuth providers like Google and GitHub. This involves obtaining API credentials (Client ID, Client Secret) from Google and GitHub, adding them as environment variables (e.g., `GOOGLE_CLIENT_ID`, `GOOGLE_CLIENT_SECRET`), and configuring BetterAuth with these providers and their respective callback URLs.
- [X] T025 [US2] Create authentication middleware `src/middleware/auth.ts` for protected routes
  * **Description**: Create a Next.js middleware file (e.g., `src/middleware.ts` or `src/middleware/auth.ts` if using a modular approach). This middleware will check the authentication status of incoming requests using BetterAuth's session data. For protected routes, it will redirect unauthenticated users to the login page or deny access if authorization roles are not met. This enforces access control across the application.
- [X] T026 [US2] Build login page UI (`src/app/login/page.tsx`) with email/password form and validation
  * **Description**: Create the login page component at `src/app/login/page.tsx`. This page will feature an email and password input form, client-side validation using React Hook Form and Zod, and a submission handler that calls the BetterAuth login API route. Utilize shadcn/ui components for a consistent UI.
- [X] T027 [US2] Build registration page UI (`src/app/register/page.tsx`) with validation
  * **Description**: Create the registration page component at `src/app/register/page.tsx`. This page will include a form for user registration (email, password, confirm password), client-side validation using React Hook Form and Zod, and a submission handler that calls the BetterAuth registration API route. Utilize shadcn/ui components for styling.
- [X] T028 [US2] Create auth provider component `src/components/AuthProvider.tsx` for client-side session
  * **Description**: Create a React context provider `src/components/AuthProvider.tsx`. This component will manage the client-side authentication state, using BetterAuth's client-side hooks or functions to check session status, provide user information, and offer login/logout methods. It will wrap the application's layout to make the authentication state globally available. management
- [X] T029 [US2] Add "Sign In" and "Sign Up" buttons to navigation `src/components/Navbar.tsx`
  * **Description**: Modify `src/components/Navbar.tsx` to include "Sign In" and "Sign Up" buttons. These buttons should conditionally render based on the user's authentication status (obtained from the `AuthProvider`). When clicked, they should navigate to the `/login` and `/register` pages, respectively.
- [X] T030 [US2] Implement user profile dropdown menu `src/components/UserMenu.tsx`
  * **Description**: Create a `src/components/UserMenu.tsx` component. This component will display a dropdown menu in the navigation bar when a user is authenticated. It should show the user's name/email and provide links to their profile, settings, and a logout option. Utilize shadcn/ui's `DropdownMenu` component for this.
- [X] T031 [US2] Build account settings page `src/app/settings/page.tsx`
  * **Description**: Create the account settings page component at `src/app/settings/page.tsx`. This page will provide forms for authenticated users to update their profile information (e.g., name, email) and change their password. It should integrate with BetterAuth's user management APIs and include appropriate client-side validation.
- [X] T032 [US2] Add logout functionality to `src/components/UserMenu.tsx`
  * **Description**: Implement a "Logout" button or menu item within `src/components/UserMenu.tsx`. This action will call the BetterAuth logout API endpoint (e.g., `/api/auth/logout`) to terminate the user's session. Upon successful logout, the user should be redirected to the login page or homepage.
- [X] T033 [US2] Implement session refresh on page load
  * **Description**: Integrate BetterAuth's session refresh mechanism into the application's main layout or `AuthProvider`. On each page load, check the session status and trigger a refresh if the current session is valid but nearing expiration, extending the user's login without explicit action.
- [X] T034 [US2] Add session expiry handling with redirect
  * **Description**: Implement logic within the `AuthProvider` or global error handler to detect session expiry. When a session expires, clear the local authentication state and redirect the user to the login page, optionally with a message indicating that their session has expired. This ensures that unauthorized users are promptly prompted to re-authenticate. logic
- [X] T035 [US2] Create protected route wrapper component `src/components/ProtectedRoute.tsx`
  * **Description**: Develop a React component `src/components/ProtectedRoute.tsx`. This component will accept children and, if the user is not authenticated (checked via the `AuthProvider`), it will redirect them to the login page. Authenticated users will see the wrapped content. This component provides a declarative way to protect client-side routes.
- [X] T036 [US2] Test session persistence across browser tabs
  * **Description**: Manually test session persistence by logging in to the application and then opening new browser tabs or windows to the application's protected routes. Verify that the authenticated session remains active across all tabs without requiring re-login. This confirms correct session cookie configuration and client-side session management.
- [X] T037 [US2] Implement "Remember Me" functionality for login
  * **Description**: Add a "Remember Me" checkbox to the login form (`src/app/login/page.tsx`). When checked, this option should configure BetterAuth to create a persistent session, typically by extending the cookie expiration time, allowing the user to remain logged in across browser sessions. The specific implementation will depend on BetterAuth's configuration options for persistent sessions.

---

## Phase 3: RAG Pipeline Development (US1 - Conversational AI for Documentation)

**Goal**: Build and deploy the Retrieval Augmented Generation system

### Tests for User Story 1

- [X] T038 [P] [US1] Unit test markdown chunking logic in `tests/rag/chunking.test.ts`
  * **Description**: Create a test file `tests/rag/chunking.test.ts`. Write unit tests to verify the markdown chunking utility (`src/utils/chunker.ts`). Tests should cover different markdown structures, header preservation, token limits (e.g., 800-1000 tokens per chunk), and overlap (e.g., 200 tokens) to ensure accurate document processing for the RAG pipeline.
- [X] T039 [P] [US1] Unit test embedding generation function in `tests/rag/embedding.test.ts`
  * **Description**: Create a test file `tests/rag/embedding.test.ts`. Write unit tests to verify the embedding generation function (`src/services/embedding.ts`). Tests should cover successful embedding generation, error handling for invalid inputs or API failures, and ensuring the output embeddings are of the correct dimension (768 for Gemini).
- [X] T040 [P] [US1] Integration test Qdrant upsert and search functionality in `tests/integration/qdrant.test.ts`
  * **Description**: Create an integration test file `tests/integration/qdrant.test.ts`. These tests will interact with the running Qdrant instance. They should verify that documents can be successfully upserted with their embeddings and metadata, and that vector search queries return relevant results with appropriate similarity scores (e.g., above 0.7). Test cases should include queries with metadata filters.
- [X] T041 [P] [US1] Integration test full RAG query pipeline in `tests/integration/rag_pipeline.test.ts`
  * **Description**: Create an integration test file `tests/integration/rag_pipeline.test.ts`. This test will simulate a user query and verify the entire RAG pipeline, including query embedding, vector search, context assembly, and LLM response generation. Test cases should assert on the accuracy of the generated responses, presence of citations, and adherence to performance expectations.

### Implementation for User Story 1

- [X] T042 [US1] Write script `scripts/ingest_docs.ts` to scan `humanoid-robotics-book/docs/` folder
  * **Description**: Create a TypeScript script `scripts/ingest_docs.ts`. This script will be responsible for recursively scanning the `humanoid-robotics-book/docs/` directory to find all `.mdx` and `.md` files. This is the first step in the document ingestion pipeline.
- [X] T043 [US1] Implement markdown-aware chunking `src/utils/chunker.ts` (800-1000 tokens, 200 overlap, headers)
  * **Description**: Create a utility file `src/utils/chunker.ts`. Implement a function that takes markdown content as input and splits it into smaller, semantically coherent chunks. The chunking logic should be markdown-aware, respecting headers, and ensure chunks are within the specified token range (800-1000 tokens) with a defined overlap (200 tokens) between consecutive chunks. This is crucial for effective RAG.
- [X] T044 [US1] Add metadata extraction `src/utils/metadata.ts` (module, section, file path, headings)
  * **Description**: Create a utility file `src/utils/metadata.ts`. This module will contain functions to extract metadata from each document (or chunk), such as the module name, section title, original file path, and heading hierarchy. This metadata will be crucial for filtering Qdrant search results and generating accurate citations.
- [ ] T045 [US1] Test chunking on sample documents and verify quality
- [ ] T046 [US1] Set up Gemini API client `src/services/gemini.ts` with API key
- [ ] T047 [US1] Implement batch embedding generation function `src/services/embedding.ts`
- [ ] T048 [US1] Process all chunks and generate embeddings using `scripts/ingest_docs.ts`
- [ ] T049 [US1] Add error handling and retry logic for embedding generation
- [ ] T050 [US1] Log embedding generation progress to console/file
- [ ] T051 [US1] Write upsert function `src/services/qdrant.ts` to store vectors in Qdrant
- [ ] T052 [US1] Batch upload embeddings with metadata to Qdrant using `scripts/ingest_docs.ts`
- [ ] T053 [US1] Verify data integrity in Qdrant
- [ ] T054 [US1] Create search test queries to validate retrieval in `src/services/qdrant.ts`
- [ ] T055 [US1] Document ingestion process in `docs/Ingestion_Process.md`
- [ ] T056 [US1] Build API route `/api/chat` for chat queries in `src/pages/api/chat.ts`
- [ ] T057 [US1] Implement query embedding generation in `src/services/embedding.ts`
- [ ] T058 [US1] Write vector search function `src/services/qdrant.ts` (top 5, score > 0.7, metadata filters)
- [ ] T059 [US1] Create context assembly logic `src/services/context.ts` (chunks + history + system prompt)
- [ ] T060 [US1] Integrate Gemini 1.5 Pro for response generation `src/services/gemini.ts` (temperature, max tokens)
- [ ] T061 [US1] Parse and format LLM response `src/utils/response_formatter.ts`
- [ ] T062 [US1] Extract citations from metadata
- [ ] T063 [US1] Format code blocks in responses
- [ ] T064 [US1] Add "Learn More" links to book sections in responses
- [ ] T065 [US1] Store query, response, sources in `chat_history` table in PostgreSQL
- [ ] T066 [US1] Implement conversation context retrieval (last 3 turns)
- [ ] T067 [US1] Add conversation reset endpoint `/api/chat/reset`

---

## Phase 4: Chat Interface (US1 - Conversational AI for Documentation)

**Goal**: Build intuitive and responsive chat widget

### Implementation for User Story 1

- [ ] T068 [US1] Create floating chat button component `src/components/ChatButton.tsx`
- [ ] T069 [US1] Build collapsible chat window component `src/components/ChatWindow.tsx` (400x600px)
- [ ] T070 [US1] Design message bubbles (`src/components/MessageBubble.tsx`) for user vs assistant
- [ ] T071 [US1] Add typing indicator `src/components/TypingIndicator.tsx` for loading state
- [ ] T072 [US1] Implement auto-scroll to latest message in `src/components/ChatWindow.tsx`
- [ ] T073 [US1] Add dark mode support to chat components
- [ ] T074 [US1] Build textarea `src/components/ChatInput.tsx` with auto-resize and send button
- [ ] T075 [US1] Implement Enter key handler for sending messages
- [ ] T076 [US1] Implement character counter (max 500 chars) for chat input
- [ ] T077 [US1] Disable input during loading state
- [ ] T078 [US1] Add input validation and error messages for chat input
- [ ] T079 [US1] Parse markdown in assistant responses (`src/components/MessageBubble.tsx`)
- [ ] T080 [US1] Syntax highlight code blocks (integrate Prism.js or similar)
- [ ] T081 [US1] Render citations as clickable links
- [ ] T082 [US1] Add copy button for code snippets
- [ ] T083 [US1] Display "Learn More" links to book sections
- [ ] T084 [US1] Add suggested questions on first open
- [ ] T085 [US1] Implement "Regenerate response" button
- [ ] T086 [US1] Add thumbs up/down feedback buttons
- [ ] T087 [US1] Show source citations in expandable section
- [ ] T088 [US1] Add "Clear conversation" button
- [ ] T089 [US1] Implement chat export as text/PDF

---

## Phase 5: User Experience Features (US3 - Personalized Learning)

**Goal**: Personalization and engagement enhancements

### Tests for User Story 3

- [ ] T090 [P] [US3] Unit test user preference management in `tests/user/preferences.test.ts`
- [ ] T091 [P] [US3] Integration test bookmark CRUD operations in `tests/integration/bookmarks.test.ts`
- [ ] T092 [P] [US3] Integration test reading progress tracking in `tests/integration/progress.test.ts`

### Implementation for User Story 3

- [ ] T093 [US3] Build preferences schema (`src/types/preferences.ts`)
- [ ] T094 [US3] Create settings page UI `src/app/settings/preferences/page.tsx`
- [ ] T095 [US3] Implement preference save/load logic `src/services/preferences.ts`
- [ ] T096 [US3] Adjust AI prompts based on response complexity setting
- [ ] T097 [US3] Test preference persistence
- [ ] T098 [US3] Add bookmark button to chat responses and book sections
- [ ] T099 [US3] Create bookmarks page `src/app/bookmarks/page.tsx` to view saved content
- [ ] T100 [US3] Implement bookmark search and filtering in `src/services/bookmarks.ts`
- [ ] T101 [US3] Add delete and organize functionality for bookmarks
- [ ] T102 [US3] Show bookmark count in navigation `src/components/Navbar.tsx`
- [ ] T103 [US3] Track module completion by page visits in `src/services/progress.ts`
- [ ] T104 [US3] Display progress bar in navigation `src/components/Navbar.tsx`
- [ ] T105 [US3] Create progress dashboard page `src/app/progress/page.tsx`
- [ ] T106 [US3] Award badges for milestones (e.g., "Completed Module 1")
- [ ] T107 [US3] Show recommended next topics in progress dashboard
- [ ] T108 [US3] Build semantic search page `src/app/search/page.tsx` (separate from chat)
- [ ] T109 [US3] Implement search results page with vector ranking
- [ ] T110 [US3] Add "Related Topics" sidebar to book pages
- [ ] T111 [US3] Create "Trending Questions" widget
- [ ] T112 [US3] Show "Recently Viewed" section

---

## Phase 6: Smart Task Modules (US1 - Conversational AI for Documentation)

**Goal**: Specialized AI assistants for specific use cases

### Implementation for User Story 1

- [ ] T113 [US1] Create `/api/quick-reference` endpoint in `src/pages/api/quick-reference.ts`
- [ ] T114 [US1] Optimize prompt for concise definitions for quick reference module
- [ ] T115 [US1] Add keyboard shortcut (Ctrl+K) to open quick reference modal
- [ ] T116 [US1] Build instant lookup modal UI `src/components/QuickReferenceModal.tsx`
- [ ] T117 [US1] Test quick reference with common queries (e.g., "DH parameters")
- [ ] T118 [US1] Create `/api/generate-code` endpoint in `src/pages/api/generate-code.ts`
- [ ] T119 [US1] Design prompt for ROS2 boilerplate generation
- [ ] T120 [US1] Add code templates for common patterns (publisher, subscriber, service)
- [ ] T121 [US1] Build code generator UI `src/components/CodeGenerator.tsx` with parameter inputs
- [ ] T122 [US1] Add "Copy to Clipboard" and "Download" buttons for generated code
- [ ] T123 [US1] Create `/api/debug` endpoint in `src/pages/api/debug.ts` (Problem Solver)
- [ ] T124 [US1] Design diagnostic prompt with step-by-step guidance
- [ ] T125 [US1] Build troubleshooting wizard UI `src/components/TroubleshootingWizard.tsx`
- [ ] T126 [US1] Test problem solver with common errors from book content
- [ ] T127 [US1] Add links to relevant debugging sections

---

## Phase 7: Performance Optimization

**Purpose**: Achieve <3s response time and smooth UX

- [ ] T128 Set up Redis instance (local + production)
- [ ] T129 Implement embedding cache (query → embedding) in `src/services/cache.ts`
- [ ] T130 Cache popular query responses (1-hour TTL) in `src/services/cache.ts`
- [ ] T131 Add cache invalidation on content updates
- [ ] T132 Monitor cache hit rate with analytics
- [ ] T133 Profile vector search performance
- [ ] T134 Tune Qdrant HNSW parameters (M, ef_construct)
- [ ] T135 Implement parallel embedding generation (debounced)
- [ ] T136 Add request deduplication for identical queries
- [ ] T137 Optimize Prisma queries with proper indexing
- [ ] T138 Lazy load chat widget component (`src/components/ChatButton.tsx`)
- [ ] T139 Code split routes and heavy dependencies (Next.js config)
- [ ] T140 Optimize bundle size (analyze with `next/bundle-analyzer`)
- [ ] T141 Add loading skeletons for better perceived performance
- [ ] T142 Implement progressive image loading

---

## Phase 8: Testing & Quality Assurance

**Purpose**: Ensure reliability and catch regressions

- [ ] T143 Write unit tests for authentication utilities in `tests/auth/`
- [ ] T144 Write unit tests for RAG pipeline functions (chunking, embedding, search) in `tests/rag/`
- [ ] T145 Write API route handlers with mocked dependencies in `tests/api/`
- [ ] T146 Achieve >80% code coverage on critical paths
- [ ] T147 Set up Jest and React Testing Library
- [ ] T148 Integration test complete chat flow (query → response) in `tests/integration/chat_flow.test.ts`
- [ ] T149 Integration test authentication flow (signup → login → logout) in `tests/integration/auth_flow.test.ts`
- [ ] T150 Integration test bookmark CRUD operations in `tests/integration/bookmarks.test.ts`
- [ ] T151 Verify RAG retrieval accuracy with test queries
- [ ] T152 Test rate limiting enforcement
- [ ] T153 Set up Playwright for E2E tests
- [ ] T154 E2E test user registration and first chat interaction
- [ ] T155 E2E test multi-turn conversation context
- [ ] T156 E2E test mobile responsive behavior
- [ ] T157 E2E test dark mode switching
- [ ] T158 Manual QA: Test all features in Chrome, Firefox, Safari
- [ ] T159 Manual QA: Test on mobile devices (iOS + Android)
- [ ] T160 Manual QA: Verify accessibility with screen reader
- [ ] T161 Manual QA: Test edge cases (empty states, long inputs, network errors)
- [ ] T162 Manual QA: Security audit (XSS, CSRF, SQL injection attempts)

---

## Phase 9: Deployment & Monitoring

**Purpose**: Ship to production with observability

- [ ] T163 Deploy Next.js app to Vercel
- [ ] T164 Deploy PostgreSQL to Supabase or Railway
- [ ] T165 Deploy Qdrant to Qdrant Cloud or Render
- [ ] T166 Configure environment variables in Vercel
- [ ] T167 Set up custom domain and SSL
- [ ] T168 Create GitHub Actions workflow for tests
- [ ] T169 Add build validation on pull requests
- [ ] T170 Set up automatic deployment from main branch
- [ ] T171 Add deployment status notifications (Slack/Discord)
- [ ] T172 Implement rollback strategy
- [ ] T173 Set up Sentry for error tracking
- [ ] T174 Configure Vercel Analytics
- [ ] T175 Add custom analytics events (chat queries, bookmarks)
- [ ] T176 Set up uptime monitoring (UptimeRobot)
- [ ] T177 Create admin dashboard for key metrics
- [ ] T178 Write deployment guide in `docs/Deployment_Guide.md`
- [ ] T179 Document environment variable requirements in `docs/Environment_Variables.md`
- [ ] T180 Create API documentation for internal routes in `docs/API_Documentation.md`
- [ ] T181 Write user guide for chatbot features in `docs/User_Guide.md`
- [ ] T182 Add troubleshooting section to `docs/Troubleshooting.md`

---

## Phase 10: Hackathon Finalization

**Purpose**: Polish for demo and submission

- [ ] T183 Create demo script with key feature showcase
- [ ] T184 Record demo video (2-3 minutes)
- [ ] T185 Prepare slide deck with architecture diagrams
- [ ] T186 Write compelling project description
- [ ] T187 Test demo on fresh browser (no cache)
- [ ] T188 Remove console.logs and debug code from `src/`
- [ ] T189 Standardize code formatting across `src/`
- [ ] T190 Add missing JSDoc comments in `src/`
- [ ] T191 Update README with project overview
- [ ] T192 Clean up unused dependencies
- [ ] T193 Full regression test of all features
- [ ] T194 Load test with 50 concurrent users (Artillery)
- [ ] T195 Verify production environment stability
- [ ] T196 Test rollback procedure
- [ ] T197 Check all error states display properly

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on other stories

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks can run in parallel where marked [P]
- All Foundational tasks can run in parallel where marked [P] (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Story 2 Only - Authentication System)

1. Complete Phase 1: Setup
2. Complete Phase 2: Authentication System (US2)
3. **STOP and VALIDATE**: Test User Story 2 independently
4. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 2 (Authentication) → Test independently → Deploy/Demo
3. Add User Story 1 (Conversational AI + Smart Task Modules) → Test independently → Deploy/Demo
4. Add User Story 3 (Personalized Learning) → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 2 (Authentication)
   - Developer B: User Story 1 (Conversational AI + Smart Task Modules)
   - Developer C: User Story 3 (Personalized Learning)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
