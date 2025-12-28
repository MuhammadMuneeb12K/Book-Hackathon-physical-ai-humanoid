import OpenAI from "openai";

const apiKey = process.env.OPENROUTER_API_KEY;
const baseURL = process.env.OPENROUTER_BASE_URL;

if (!apiKey || !baseURL) {
  throw new Error(
    "🔥 OPENROUTER_API_KEY or OPENROUTER_BASE_URL is missing in .env.local"
  );
}

export const openaiClient = new OpenAI({
  apiKey,
  baseURL,
});
