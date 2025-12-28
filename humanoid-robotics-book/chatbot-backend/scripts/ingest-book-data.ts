import 'dotenv/config'

import { readFileSync, readdirSync, statSync } from 'fs'
import { join } from 'path'
import { qdrantClient, COLLECTION_NAME } from '../src/lib/rag/qdrant-client'
import { openaiClient } from '../src/lib/openai-client'

interface DocumentChunk {
  content: string
  metadata: {
    module: string
    file: string
    section: string
    chunkIndex: number
  }
}

function extractMarkdownContent(content: string): string {
  content = content.replace(/^---[\s\S]*?---\n/, '')
  content = content.replace(/^import .+$/gm, '')
  content = content.replace(/<!--[\s\S]*?-->/g, '')
  return content.trim()
}

function chunkText(text: string, chunkSize = 1000, overlap = 200): string[] {
  if (!text || text.length === 0) return []; // handle empty files
  if (overlap >= chunkSize) overlap = Math.floor(chunkSize / 2); // ensure valid overlap

  const chunks: string[] = [];
  let start = 0;

  while (start < text.length) {
    const end = Math.min(start + chunkSize, text.length);
    chunks.push(text.slice(start, end));

    // Advance start, prevent negative or infinite loop
    if (end === text.length) break;
    start = end - overlap;
  }
  return chunks
}

async function scanDocuments(docsPath: string): Promise<DocumentChunk[]> {
  const documents: DocumentChunk[] = []

  function scanDirectory(dirPath: string, module: string = '') {
    const items = readdirSync(dirPath)

    for (const item of items) {
      const fullPath = join(dirPath, item)
      const stat = statSync(fullPath)

      if (stat.isDirectory()) {
        const moduleName = module || item
        scanDirectory(fullPath, moduleName)
      } else if (item.endsWith('.md') || item.endsWith('.mdx')) {
        const content = readFileSync(fullPath, 'utf-8')
        const cleanContent = extractMarkdownContent(content)
        const chunks = chunkText(cleanContent)

        chunks.forEach((chunk, index) => {
          documents.push({
            content: chunk,
            metadata: {
              module: module || 'root',
              file: item,
              section: item.replace(/\.(md|mdx)$/, ''),
              chunkIndex: index,
            },
          })
        })
      }
    }
  }

  scanDirectory(docsPath)
  return documents
}

async function generateEmbeddings(texts: string[]): Promise<number[][]> {
  const allEmbeddings: number[][] = [];
  const batchSize = 100; // OpenAI's API can handle large batches

  for (let i = 0; i < texts.length; i += batchSize) {
    const batch = texts.slice(i, i + batchSize);
    console.log(`Generating embeddings for batch ${i / batchSize + 1}/${Math.ceil(texts.length / batchSize)}...`);

    const response = await openaiClient.embeddings.create({
      model: "text-embedding-3-small", // Recommended model for balance of performance and cost
      input: batch,
    });
    
    // Extract the embedding vectors from the response
    const embeddings = response.data.map(item => item.embedding);
    allEmbeddings.push(...embeddings);
  }

  return allEmbeddings;
}

async function ingestData() {
  console.log('📚 Starting book data ingestion...\n')

  const docsPath = join(process.cwd(), 'docs')

  console.log('📖 Scanning documents...')
  const documents = await scanDocuments(docsPath)
  console.log(`✅ Found ${documents.length} document chunks\n`)

  console.log('🧮 Generating embeddings...')
  const texts = documents.map((doc) => doc.content)
  const embeddings = await generateEmbeddings(texts)
  console.log(`✅ Generated ${embeddings.length} embeddings\n`)

  console.log('📤 Uploading to Qdrant...')
  const points = documents.map((doc, index) => ({
    id: index,
    vector: embeddings[index],
    payload: {
      content: doc.content,
      ...doc.metadata,
    },
  }))

  await qdrantClient.upsert(COLLECTION_NAME, {
    wait: true,
    points: points,
  })

  console.log('✅ Data uploaded successfully!\n')
  console.log('🎉 Ingestion complete!')
  console.log(`📊 Total chunks: ${documents.length}`)
}

ingestData().catch(console.error)
