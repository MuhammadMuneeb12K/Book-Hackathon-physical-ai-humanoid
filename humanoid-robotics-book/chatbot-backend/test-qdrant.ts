import 'dotenv/config';
import fetch from 'node-fetch';

// Polyfill fetch for environments that might not have it globally
if (!globalThis.fetch) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  (globalThis as any).fetch = fetch;
}

// Use an async IIFE (Immediately Invoked Function Expression) to use async/await
(async () => {
  try {
    console.log("Testing connection to Qdrant...");
    const response = await fetch(process.env.QDRANT_URL! + '/collections', {
      headers: { 'api-key': process.env.QDRANT_API_KEY! },
    });

    if (!response.ok) {
      throw new Error(`Qdrant API responded with status: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    console.log("✅ Successfully connected to Qdrant. Collections:", data);
  } catch (error: unknown) {
    console.error("🔴 Failed to connect to Qdrant:");
    if (error instanceof Error) {
      console.error(error.message);
    } else {
      console.error("An unknown error occurred:", error);
    }
  }
})();
