# Feature Specification: RAG Chatbot for Docusaurus Book

**Feature Branch**: `1-rag-chatbot`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "## Project Overview
Build a Retrieval-Augmented Generation (RAG) chatbot that integrates seamlessly into a Docusaurus-based book website. The chatbot must answer questions about the book's content and support text-selection-based queries.

## Technology Stack
- **Backend Framework**: FastAPI (Python)
- **AI/LLM**: OpenAI GPT models via OpenAI SDK
- **context7 mcp server
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Relational Database**: Neon Serverless Postgres
- **Frontend Integration**: React component embedded in Docusaurus
- **Deployment**: Backend on a suitable platform (e.g., Vercel, Railway, or Render)

## Core Features

### 1. Document Ingestion & Vector Storage
- **Data Source**: Extract all markdown content from the Docusaurus `/docs` folder
- **Text Processing**:
  - Split documents into semantic chunks (500-1000 tokens with 100-token overlap)
  - Preserve document hierarchy and metadata (chapter, section, page URL)
  - Generate embeddings using OpenAI's `text-embedding-3-small` model
- **Vector Storage**:
  - Store embeddings in Qdrant Cloud with metadata
  - Create collection with proper indexing
  - Implement upsert functionality for content updates

### 2. RAG Query System
- **Query Processing**:
  - Accept user questions via REST API endpoint
  - Generate query embeddings
  - Perform similarity search in Qdrant (top 5-10 relevant chunks)
  - Retrieve context with source references
- **Response Generation**:
  - Use OpenAI ChatKit/Agents SDK to generate answers
  - Include retrieved context in the prompt
  - Cite sources with chapter/section references
  - Handle questions that cannot be answered from the book content

### 3. Text Selection Feature
- **Frontend Selection Handler**:
  - Detect when user highlights/selects text on the page
  - Show contextual \"Ask AI about this\" button near selection
  - Send selected text as context with user's question
- **Backend Processing**:
  - Accept both query and selected text
  - Prioritize selected text as primary context
  - Augment with additional relevant chunks if needed
  - Generate focused answers based on selection

### 4. Chat Interface Component
- **UI Requirements**:
  - Floating chat widget (bottom-right corner)
  - Collapsible/expandable interface
  - Message history within session
  - Loading states and error handling
  - Source citations as clickable links to book sections
- **Styling**:
  - Match Docusaurus theme (light/dark mode support)
  - Responsive design (mobile-friendly)
  - Smooth animations

### 5. FastAPI Backend Structure
```
/api/v1/chat
  POST /query - Main chat endpoint
  POST /query-selection - Query with selected text
  GET /health - Health check

/api/v1/admin
  POST /ingest - Trigger document ingestion
  GET /stats - Get collection statistics
```

### 6. Database Schema (Neon Postgres)
```sql
-- Chat history table
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY,
    created_at TIMESTAMP,
    user_agent TEXT
);

CREATE TABLE chat_messages (
    id UUID PRIMARY KEY,
    session_id UUID REFERENCES chat_sessions(id),
    role VARCHAR(20), -- 'user' or 'assistant'
    content TEXT,
    sources JSONB, -- Store source references
    created_at TIMESTAMP
);

-- Document tracking
CREATE TABLE document_versions (
    id SERIAL PRIMARY KEY,
    doc_path TEXT,
    last_updated TIMESTAMP,
    chunk_count INTEGER
);
```"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Book Content (Priority: P1)

As a reader browsing the Docusaurus-based book website, I want to ask questions about the book content and receive accurate answers with source citations, so that I can quickly find information without manually searching through the documentation.

**Why this priority**: This is the core functionality that provides immediate value to users by enabling them to get answers to their questions quickly.

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that the chatbot responds with accurate answers and proper source citations.

**Acceptance Scenarios**:

1. **Given** I am on a book page with the chat widget available, **When** I type a question about the book content, **Then** I receive an accurate answer with relevant source citations.
2. **Given** I ask a question that cannot be answered from the book content, **When** I submit the question, **Then** I receive a response indicating that the question cannot be answered based on the book content.

---

### User Story 2 - Ask Questions About Selected Text (Priority: P2)

As a reader who has highlighted text on a book page, I want to ask questions specifically about the selected text using a contextual button, so that I can get more detailed information about specific content I'm reading.

**Why this priority**: This provides an enhanced user experience by allowing contextual questions about specific content.

**Independent Test**: Can be fully tested by selecting text on a page, clicking the "Ask AI about this" button, and verifying that responses are contextually relevant to the selected text.

**Acceptance Scenarios**:

1. **Given** I have selected text on a book page, **When** I click the contextual "Ask AI about this" button and ask a question, **Then** I receive an answer that prioritizes context from the selected text.

---

### User Story 3 - View Chat History and Source Citations (Priority: P2)

As a user who has asked multiple questions, I want to see my conversation history within the session and clickable source citations, so that I can reference previous answers and navigate to relevant book sections.

**Why this priority**: This enhances the user experience by providing continuity and easy access to source material.

**Independent Test**: Can be fully tested by asking multiple questions, viewing the chat history, and clicking on source citations to navigate to relevant sections.

**Acceptance Scenarios**:

1. **Given** I have asked multiple questions in a session, **When** I view the chat interface, **Then** I can see the complete conversation history.
2. **Given** I see source citations in the chat response, **When** I click on a citation link, **Then** I am navigated to the relevant section of the book.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a floating chat widget that can be collapsed and expanded on Docusaurus book pages
- **FR-002**: System MUST accept user questions via REST API and generate appropriate responses based on book content
- **FR-003**: System MUST retrieve relevant document chunks from vector storage to support answer generation
- **FR-004**: System MUST generate and store embeddings for book content in the vector database
- **FR-005**: System MUST preserve document hierarchy and metadata (chapter, section, page URL) during ingestion
- **FR-006**: System MUST provide source citations with clickable links to relevant book sections in chat responses
- **FR-007**: System MUST support text selection functionality with a contextual "Ask AI about this" button
- **FR-008**: System MUST prioritize selected text as primary context when processing text selection queries
- **FR-009**: System MUST maintain chat session history for the duration of the user's visit
- **FR-010**: System MUST handle questions that cannot be answered from book content appropriately
- **FR-011**: System MUST support light and dark mode themes matching the Docusaurus site
- **FR-012**: System MUST be responsive and work on mobile devices
- **FR-013**: System MUST provide health check endpoint for monitoring
- **FR-014**: System MUST provide admin endpoint to trigger document ingestion
- **FR-015**: System MUST store chat session data including messages and source references in the database

### Key Entities

- **Chat Session**: Represents a user's conversation with the chatbot during a visit, containing metadata like creation time and user agent
- **Chat Message**: Represents an individual message in a conversation, including role (user/assistant), content, and source citations
- **Document Chunk**: Represents a segment of book content with associated metadata (chapter, section, page URL) and vector embeddings
- **Document Version**: Tracks the version and update status of book documents to enable content synchronization

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can ask questions about book content and receive accurate answers with source citations in under 5 seconds
- **SC-002**: The system successfully answers at least 80% of relevant questions based on book content
- **SC-003**: At least 70% of users who interact with the chatbot find the answers helpful according to user feedback
- **SC-004**: The chatbot correctly handles questions outside the book scope by indicating when content is not available
- **SC-005**: Document ingestion process successfully processes all markdown files from the Docusaurus docs folder
- **SC-006**: The floating chat widget is accessible and functional on all book pages across different devices and screen sizes
- **SC-007**: Text selection feature is available and responsive on all supported browsers