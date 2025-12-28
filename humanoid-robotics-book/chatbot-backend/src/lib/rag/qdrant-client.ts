
import { QdrantClient } from '@qdrant/js-client-rest'

const qdrantUrl = process.env.QDRANT_URL || 'http://localhost:6333'
const qdrantApiKey = process.env.QDRANT_API_KEY

export const qdrantClient = new QdrantClient({
  url: qdrantUrl,
  apiKey: qdrantApiKey,
})

export const COLLECTION_NAME = process.env.QDRANT_COLLECTION || 'humanoid_robotics_book'
export const VECTOR_SIZE = 768
