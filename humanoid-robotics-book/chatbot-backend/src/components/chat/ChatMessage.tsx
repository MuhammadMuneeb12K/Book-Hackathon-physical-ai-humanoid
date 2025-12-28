// src/components/chat/ChatMessage.tsx
'use client';

import { useState, useEffect } from 'react';

// Define the shape of a source object
interface Source {
  module: string;
  file: string;
  section:string;
  content: string;
}

// Define the shape of a message object
interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

interface Props {
  message: Message;
}

export default function ChatMessage({ message }: Props) {
  const [showSources, setShowSources] = useState(false);
  // FIX: Hydration Mismatch for Timestamps
  // The server and client might have different timezones or locales, causing
  // `toLocaleTimeString()` to render different strings.
  // To fix this, we'll render the timestamp only on the client after hydration.
  const [formattedTime, setFormattedTime] = useState('');

  // By extracting the timestamp, we make the dependency clearer to the linter.
  const timestamp = message.timestamp;
  useEffect(() => {
    // This effect runs only on the client, after the component has mounted.
    // We can safely use browser-specific APIs like `toLocaleTimeString` here.
    setFormattedTime(new Date(timestamp).toLocaleTimeString());
  }, [timestamp]); // Re-run if the timestamp changes

  const isUser = message.role === 'user';

  return (
    <div className={`flex animate-fadeIn ${isUser ? 'justify-end' : 'justify-start'}`}>
      <div className="max-w-[85%]">
        {/* Avatar and Name */}
        <div className={`flex items-center gap-3 mb-2 ${isUser ? 'justify-end' : ''}`}>
          <div
            className={`w-8 h-8 rounded-full flex items-center justify-center text-lg shadow-md ${isUser
              ? 'bg-gradient-to-br from-purple-500 to-blue-500'
              : 'bg-gradient-to-br from-cyan-500 to-blue-500'
            }`}
          >
            {isUser ? '👤' : '🤖'}
          </div>
          <span className="text-gray-300 font-medium">
            {isUser ? 'You' : 'AI Assistant'}
          </span>
        </div>

        {/* Message Bubble */}
        <div
          className={`rounded-2xl px-5 py-4 shadow-lg ${isUser
            ? 'bg-gradient-to-r from-purple-600 to-blue-600 text-white'
            : 'bg-white/10 text-white border border-white/20 backdrop-blur-md'
          }`}
        >
          {/* Use a regular expression to format code blocks within the message content */}
          <div
            className="prose prose-invert prose-sm max-w-none"
            dangerouslySetInnerHTML={{
              __html: message.content.replace(/```(\w+)?\n([\s\S]*?)```/g,
              '<pre><code class="language-$1">$2</code></pre>'),
            }}
          />
        </div>

        {/* Sources Toggle */}
        {!isUser && message.sources && message.sources.length > 0 && (
          <div className="mt-3 text-left">
            <button
              onClick={() => setShowSources(!showSources)}
              className="text-sm text-gray-400 hover:text-gray-200 flex items-center gap-2 font-medium transition-colors"
            >
              <span>📚</span>
              <span>{showSources ? 'Hide' : 'Show'} {message.sources.length} source(s)</span>
              <span className={`transform transition-transform ${showSources ? 'rotate-90' : ''}`}>▶</span>
            </button>
            {showSources && (
              <div className="mt-2 space-y-2 animate-fadeIn">
                {message.sources.slice(0, 3).map((source, idx) => (
                  <div key={idx} className="bg-white/5 border border-white/10 rounded-lg p-3 hover:bg-white/10 transition-colors text-xs">
                    <div className="flex items-center gap-2 mb-2">
                      <span className="px-2 py-1 bg-purple-500/30 text-purple-200 rounded text-xs font-bold">
                        {source.module}
                      </span>
                      <span className="text-gray-400">{source.section}</span>
                    </div>
                    <p className="text-gray-400 line-clamp-3">{source.content}</p>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}

        {/* Timestamp */}
        <p className="text-xs text-gray-500 mt-2 text-left ml-2">
          {/* The formattedTime will be empty on server render and initial client render, matching perfectly. */}
          {/* It will be populated on the client after hydration, avoiding the mismatch error. */}
          {formattedTime}
        </p>
      </div>
    </div>
  );
}