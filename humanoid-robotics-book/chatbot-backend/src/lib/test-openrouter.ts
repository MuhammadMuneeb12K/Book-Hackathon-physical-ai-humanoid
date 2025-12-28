import { openaiClient } from "@/lib/openai-client";

async function main() {
  console.log("Running OpenRouter test...");

  // Check if the API key was loaded by the --require flag
  if (!process.env.OPENROUTER_API_KEY) {
    throw new Error(
      "🔴 FATAL: OPENROUTER_API_KEY is not defined. Ensure it is set in your .env.local file and you are running this script via 'npm run test:openrouter'."
    );
  }

  try {
    const completion = await openaiClient.chat.completions.create({
      // When using OpenRouter, the model name is 'vendor/model-name'
      // Using a free, small model for testing.
      model: "mistralai/mistral-7b-instruct", 
      messages: [
        {
          role: "system",
          content: "You are a helpful assistant.",
        },
        {
          role: "user",
          content: "Explain the importance of loading .env files in a Node.js script.",
        },
      ],
      temperature: 0.7,
    });

    const answer = completion.choices[0]?.message?.content;

    if (!answer) {
      throw new Error("The model returned an empty response.");
    }

    console.log("✅ Success! Model Response:");
    console.log("--------------------------");
    console.log(answer);
    console.log("--------------------------");
  } catch (error: unknown) {
    console.error("🔴 An error occurred during the API call:");
    
    // Type-safe way to check for nested properties without using 'any'
    if (
      typeof error === "object" &&
      error !== null &&
      "response" in error
    ) {
      const response = (error as { response?: unknown }).response;
      if (
        typeof response === "object" &&
        response !== null &&
        "data" in response
      ) {
        console.error(JSON.stringify((response as { data: unknown }).data, null, 2));
      }
    } else if (error instanceof Error) {
      console.error(error.message);
    } else {
      console.error("An unknown error occurred:", error);
    }
    process.exit(1);
  }
}

main();
