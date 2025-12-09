# Quickstart Guide: RAG Chatbot for Docusaurus Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus development)
- OpenAI API key
- Qdrant Cloud account
- Neon PostgreSQL account

## Setup Steps

### 1. Clone and Initialize Repository

```bash
git clone <your-repo-url>
cd <repo-name>
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create and activate virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cp .env.example .env
# Edit .env with your API keys and connection strings
```

### 3. Environment Variables

Create `.env` file with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
CORS_ORIGINS=http://localhost:3000,https://yourdomain.com
```

### 4. Initialize Vector Database

```bash
# Run the document ingestion script to populate Qdrant
python scripts/ingest_documents.py
```

### 5. Run Backend Server

```bash
# From the backend directory
uvicorn src.main:app --reload --port 8000
```

### 6. Frontend Integration

```bash
# Navigate to your Docusaurus project
cd path/to/your/docusaurus

# Install the chatbot component
npm install path/to/chatbot/component

# Add to docusaurus.config.js
module.exports = {
  // ... existing config
  scripts: [
    {
      src: '/chatbot-init.js',
      async: true,
    },
  ],
};
```

## Code Examples

### Basic API Usage

```python
import openai
from openai import OpenAI
from qdrant_client import QdrantClient

# Initialize clients
client = OpenAI()
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
```

### Running the Application

```bash
# Backend (in backend/ directory)
uvicorn src.main:app --reload --port 8000

# Frontend (in Docusaurus directory)
npm run start
```

## Dependencies

### Backend (requirements.txt)
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
openai==1.3.0
qdrant-client==1.7.0
psycopg2-binary==2.9.9
sqlalchemy==2.0.23
python-dotenv==1.0.0
langchain==0.1.0
langchain-openai==0.0.2
pydantic==2.5.0
```

### Frontend (package.json)
```json
{
  "dependencies": {
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  }
}
```

## First Run Checklist

- [ ] Environment variables configured
- [ ] Qdrant collection created and populated
- [ ] Database connection established
- [ ] API endpoints accessible at http://localhost:8000
- [ ] Frontend component integrated with Docusaurus
- [ ] Test query returns results with source citations