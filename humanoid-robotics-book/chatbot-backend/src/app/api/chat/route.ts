import { NextResponse } from "next/server";
import { openaiClient } from "@/lib/openai-client";

const model = process.env.OPENROUTER_MODEL || "gpt-4o-mini";
const systemPrompt = process.env.SYSTEM_PROMPT || "You are a helpful assistant.";

/**
 * POST /api/chat
 * Accepts a user message and returns a response from OpenRouter.
 */
export async function POST(req: Request) {
  try {
    const { message } = await req.json();

    if (!message) {
      return NextResponse.json(
        { error: "No message provided." },
        { status: 400 }
      );
    }

    const completion = await openaiClient.chat.completions.create({
      model: model,
      messages: [
        { role: "system", content: systemPrompt },
        { role: "user", content: message },
      ],
      temperature: 0.4,
    });

    const answer = completion.choices[0]?.message?.content;

    if (!answer) {
      return NextResponse.json(
        { error: "The model returned an empty response." },
        { status: 500 }
      );
    }

    return NextResponse.json({ answer });
  } catch (error: unknown) {
    console.error("OpenRouter Error:", error);
    
    let errorMessage = "An unknown error occurred.";
    if (error instanceof Error) {
      errorMessage = `An error occurred while communicating with the AI model: ${error.message}`;
    }

    return NextResponse.json(
      { error: errorMessage },
      { status: 500 }
    );
  }
}
