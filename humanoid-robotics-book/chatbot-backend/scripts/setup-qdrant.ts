import "dotenv/config";
import { qdrantClient, COLLECTION_NAME, VECTOR_SIZE } from '../src/lib/rag/qdrant-client'

async function setupQdrant() {
  console.log('🔧 Setting up Qdrant collection...')

  try {
    const collections = await qdrantClient.getCollections()
    const exists = collections.collections.some((col) => col.name === COLLECTION_NAME)

    if (exists) {
      console.log(`⚠️  Collection "${COLLECTION_NAME}" already exists. Deleting...`)
      await qdrantClient.deleteCollection(COLLECTION_NAME)
    }

    await qdrantClient.createCollection(COLLECTION_NAME, {
      vectors: {
        size: VECTOR_SIZE,
        distance: 'Cosine',
      },
      optimizers_config: {
        default_segment_number: 2,
      },
      replication_factor: 1,
    })

    console.log(`✅ Collection "${COLLECTION_NAME}" created successfully!`)

    await qdrantClient.createPayloadIndex(COLLECTION_NAME, {
      field_name: 'module',
      field_schema: 'keyword',
    })

    console.log('✅ Payload indexes created')
    console.log('\n🎉 Qdrant setup complete!')
  } catch (error) {
    console.error('❌ Error setting up Qdrant:', error)
    process.exit(1)
  }
}

setupQdrant()
