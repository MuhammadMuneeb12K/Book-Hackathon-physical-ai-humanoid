import { openaiClient } from "@/lib/openai-client";

// The system prompt is now managed via the .env.local file for better configuration.
const systemPrompt = process.env.SYSTEM_PROMPT;
const model = process.env.OPENROUTER_MODEL || "gpt-4o-mini";

/**
 * Queries the OpenRouter API using the gpt-4o-mini model.
 *
 * @param {string} question - The user's question.
 * @returns {Promise<{ answer: string | null; sources: {} }>} - A promise that resolves to the answer and sources.
 */
export async function queryRAG(question: string) {
  // Guard against missing OpenRouter API key at runtime.
  if (!process.env.OPENROUTER_API_KEY || !process.env.OPENROUTER_BASE_URL) {
    console.error("🔥 FATAL: OPENROUTER_API_KEY or OPENROUTER_BASE_URL is not defined.");
    return {
      answer: "Server configuration error: The API key or base URL is missing. Please contact the administrator.",
      sources: {},
    };
  }

  try {
    const completion = await openaiClient.chat.completions.create({
      model: model, // Can be configured in .env.local
      messages: [
        {
          role: "system",
          content: systemPrompt || "You are a helpful assistant.", // Fallback prompt
        },
        {
          role: "user",
          content: question,
        },
      ],
      temperature: 0.4, // Lower temperature encourages deterministic responses
    });

    const answer = completion.choices[0]?.message?.content;

    if (!answer) {
      console.error("🔥 OpenRouter returned a completion with no content.");
      return {
        answer: "The model returned an empty response. Please try rephrasing your question.",
        sources: {},
      };
    }

    return {
      answer: answer,
      sources: {}, // Placeholder for future RAG source integration
    };
  } catch (error: unknown) {
    console.error("🔥 OPENROUTER ERROR:", error);

    let errorMessage = "An unknown error occurred while communicating with the AI model.";
    if (error instanceof Error) {
      errorMessage = `An error occurred while communicating with the AI model: ${error.message}`;
    }
    
    // Provide a more user-friendly error message
    return {
      answer: errorMessage,
      sources: {},
    };
  }
}
