# Data Model: RAG Chatbot

## Entity Relationship Diagram

```
ChatSession (1) ←→ (Many) ChatMessage
    ↓
DocumentVersion
```

## Entity Definitions

### ChatSession
- **id**: UUID (Primary Key) - Unique identifier for the session
- **created_at**: TIMESTAMP - When the session was created
- **user_agent**: TEXT - Browser/user agent information

### ChatMessage
- **id**: UUID (Primary Key) - Unique identifier for the message
- **session_id**: UUID (Foreign Key → ChatSession.id) - Links to parent session
- **role**: VARCHAR(20) - Message role ('user' or 'assistant')
- **content**: TEXT - The actual message content
- **sources**: JSONB - Source citations in JSON format
- **created_at**: TIMESTAMP - When the message was created

### DocumentVersion
- **id**: SERIAL (Primary Key) - Auto-incrementing ID
- **doc_path**: TEXT - Path to the document in the Docusaurus structure
- **last_updated**: TIMESTAMP - Last time the document was processed
- **chunk_count**: INTEGER - Number of chunks created from this document

## Vector Database Schema (Qdrant)

### Collection: book_content
- **Vectors**: 1536 dimensions (for OpenAI text-embedding-3-small)
- **Payload**:
  - `content`: TEXT - The text chunk content
  - `chapter`: TEXT - Chapter name
  - `section`: TEXT - Section name
  - `url`: TEXT - URL to the document section
  - `doc_path`: TEXT - Path to the source document
  - `metadata`: JSON - Additional metadata

## Relationships

1. **ChatSession → ChatMessage**: One-to-Many relationship
   - One chat session can contain many messages
   - Foreign key constraint ensures referential integrity

2. **DocumentVersion**: Standalone entity tracking document processing status
   - Not directly linked to chat entities but used for content synchronization

## Indexes

- ChatSession.created_at: For session cleanup and analytics
- ChatMessage.session_id: For efficient session-based queries
- ChatMessage.created_at: For message ordering and cleanup
- DocumentVersion.doc_path: For efficient document lookup
- Qdrant: Payload indexes on chapter, section, and url fields for fast filtering