# API Contract: Chat Endpoints

## Overview
Chat API endpoints for the RAG chatbot system, providing query capabilities and text selection features.

## Base URL
`/api/v1/chat`

## Endpoints

### POST /query
Main chat endpoint for asking questions about book content.

#### Request
```json
{
  "question": "What is RAG?",
  "session_id": "uuid-here"
}
```

**Request Schema:**
- `question`: string (required) - The user's question about book content
- `session_id`: string (optional) - UUID for tracking conversation history

#### Response
```json
{
  "answer": "RAG (Retrieval-Augmented Generation) is...",
  "sources": [
    {
      "chapter": "Chapter 1",
      "section": "Introduction to RAG",
      "url": "/docs/chapter-1/intro-rag"
    }
  ],
  "message_id": "uuid-here",
  "session_id": "uuid-here"
}
```

**Response Schema:**
- `answer`: string - The AI-generated response to the question
- `sources`: array of objects - Source citations for the response
  - `chapter`: string - Chapter name
  - `section`: string - Section name
  - `url`: string - URL to the source section
- `message_id`: string - UUID for the response message
- `session_id`: string - UUID for the chat session

#### Error Responses
- `400 Bad Request`: Invalid request format
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Processing error

### POST /query-selection
Endpoint for asking questions about selected text with additional context.

#### Request
```json
{
  "question": "Explain this in simpler terms",
  "selected_text": "Vector embeddings are numerical representations...",
  "page_url": "/docs/chapter-2/embeddings",
  "session_id": "uuid-here"
}
```

**Request Schema:**
- `question`: string (required) - The user's question about the selected text
- `selected_text`: string (required) - The text that was selected by the user
- `page_url`: string (required) - URL of the page where text was selected
- `session_id`: string (optional) - UUID for tracking conversation history

#### Response
```json
{
  "answer": "Vector embeddings are mathematical representations...",
  "sources": [
    {
      "chapter": "Chapter 2",
      "section": "Embeddings Explained",
      "url": "/docs/chapter-2/embeddings"
    },
    {
      "chapter": "Chapter 2",
      "section": "Advanced Embeddings",
      "url": "/docs/chapter-2/advanced-embeddings"
    }
  ],
  "message_id": "uuid-here",
  "session_id": "uuid-here"
}
```

**Response Schema:** Same as POST /query

#### Error Responses
- `400 Bad Request`: Invalid request format
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Processing error

### GET /health
Health check endpoint to verify service availability.

#### Response
```json
{
  "status": "healthy",
  "timestamp": "2025-12-10T10:30:00Z"
}
```

**Response Schema:**
- `status`: string - Service status ("healthy" or "unhealthy")
- `timestamp`: string - ISO 8601 timestamp of the check

#### Error Responses
- `503 Service Unavailable`: Service is not healthy