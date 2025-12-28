import { prisma } from '../src/lib/db/prisma'
import { qdrantClient, COLLECTION_NAME } from '../src/lib/rag/qdrant-client'

async function verify() {
  console.log('🔍 Verifying setup...\n')

  try {
    await prisma.$connect()
    console.log('✅ PostgreSQL connected')
    await prisma.$disconnect()
  } catch (error) {
    console.error('❌ PostgreSQL failed:', error)
  }

  try {
    const collections = await qdrantClient.getCollections()
    const exists = collections.collections.some(
      (col) => col.name === COLLECTION_NAME
    )
    
    if (exists) {
      const info = await qdrantClient.getCollection(COLLECTION_NAME)
      console.log(`✅ Qdrant connected - ${info.points_count} vectors stored`)
    } else {
      console.log('⚠️  Qdrant collection not found. Run: npm run qdrant:setup')
    }
  } catch (error) {
    console.error('❌ Qdrant failed:', error)
  }

  if (process.env.GOOGLE_API_KEY) {
    console.log('✅ Gemini API key configured')
  } else {
    console.error('❌ GOOGLE_API_KEY missing')
  }

  console.log('\n✨ Verification complete!')
}

verify()
