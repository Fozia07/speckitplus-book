# API Contract: Admin Endpoints

## Overview
Admin API endpoints for managing the RAG system, including document ingestion and statistics.

## Base URL
`/api/v1/admin`

## Endpoints

### POST /ingest
Trigger the document ingestion process to update the vector database with new or updated book content.

#### Request
```json
{
  "force_update": false,
  "doc_paths": ["/docs/chapter-1", "/docs/chapter-2"]
}
```

**Request Schema:**
- `force_update`: boolean (optional, default: false) - If true, reprocess all documents regardless of update status
- `doc_paths`: array of strings (optional) - Specific document paths to process; if omitted, processes all documents

#### Response
```json
{
  "status": "started",
  "job_id": "uuid-here",
  "documents_processed": 15,
  "message": "Document ingestion started successfully"
}
```

**Response Schema:**
- `status`: string - Current status of the job ("started", "completed", "failed")
- `job_id`: string - UUID for tracking the ingestion job
- `documents_processed`: number - Number of documents processed
- `message`: string - Human-readable status message

#### Error Responses
- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Invalid credentials
- `500 Internal Server Error`: Processing error

### GET /stats
Get statistics about the document collection and system status.

#### Response
```json
{
  "total_documents": 25,
  "total_chunks": 1250,
  "vector_db_status": "connected",
  "last_ingestion": "2025-12-10T09:15:00Z",
  "storage_usage": {
    "chunks": 1250,
    "total_size_mb": 15.2
  },
  "model_info": {
    "embedding_model": "text-embedding-3-small",
    "vector_dimensions": 1536
  }
}
```

**Response Schema:**
- `total_documents`: number - Total number of processed documents
- `total_chunks`: number - Total number of text chunks in vector database
- `vector_db_status`: string - Connection status to vector database
- `last_ingestion`: string - ISO 8601 timestamp of last ingestion
- `storage_usage`: object - Storage statistics
  - `chunks`: number - Number of chunks stored
  - `total_size_mb`: number - Approximate storage size in MB
- `model_info`: object - Information about the embedding model
  - `embedding_model`: string - Name of the embedding model used
  - `vector_dimensions`: number - Dimensionality of the vectors

#### Error Responses
- `401 Unauthorized`: Invalid credentials
- `500 Internal Server Error`: Processing error