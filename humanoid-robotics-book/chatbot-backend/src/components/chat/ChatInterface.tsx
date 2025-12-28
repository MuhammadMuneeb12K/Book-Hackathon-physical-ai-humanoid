// src/components/chat/ChatInterface.tsx
'use client';

import { v4 as uuidv4 } from "uuid";
import { useState, useRef, useEffect } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import SuggestedQuestions from './SuggestedQuestions';

// Define the shape of a source object, used for RAG citations
interface Source {
  file: string;
  section: string;
  // content and score could be added here in the future
}

// Define the shape of a message object
interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: Date;
}

/**
 * The main interactive chat component.
 *
 * This is a Client Component that manages all chat-related state and logic,
 * including the message list, user input, loading states, and API communication.
 */
export default function ChatInterface() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom whenever messages change
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(scrollToBottom, [messages]);

  // Handle sending a message
  const handleSendMessage = async (message: string) => {
    if (!message.trim()) return;

    const userMessage: Message = {
      id: uuidv4(),
      role: 'user',
      content: message,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message,
          conversationHistory: messages.map((m) => ({ role: m.role, content: m.content })),
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => null);
        throw new Error(errorData?.error || `API Error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: uuidv4(),
        role: 'assistant',
        content: data.answer || 'Sorry, I could not generate a response.',
        sources: data.sources || [],
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat API Error:', error);

      const errorMessage: Message = {
        id: uuidv4(),
        role: 'assistant',
        content: `⚠️ Sorry, an error occurred: ${error instanceof Error ? error.message : 'Unknown error'}`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearChat = () => setMessages([]);

  return (
    <div className="bg-white/10 backdrop-blur-xl rounded-2xl shadow-2xl border border-white/20 overflow-hidden">
      {/* Chat Header */}
      <div className="bg-gradient-to-r from-purple-600 via-blue-600 to-cyan-600 px-6 py-4 flex justify-between items-center">
        <div className="flex items-center gap-3">
          <div className="relative">
            <div className="w-3 h-3 bg-green-400 rounded-full animate-pulse"></div>
            <div className="absolute inset-0 w-3 h-3 bg-green-400 rounded-full animate-ping opacity-75"></div>
          </div>
          <span className="text-white font-bold text-lg">AI Assistant Online</span>
        </div>
        {messages.length > 0 && (
          <button
            onClick={handleClearChat}
            className="text-white/90 hover:text-white bg-white/20 hover:bg-white/30 px-3 py-1 rounded-lg transition-all font-medium text-sm"
          >
            Clear Chat
          </button>
        )}
      </div>

      {/* Messages Area */}
      <div className="h-[60vh] min-h-[400px] overflow-y-auto p-6 space-y-6">
        {messages.length === 0 ? (
          <div className="h-full flex flex-col items-center justify-center text-center">
            <div className="text-7xl mb-6">💬</div>
            <h3 className="text-white text-2xl font-bold mb-3">Ask Me Anything</h3>
            <p className="text-gray-300 mb-8">
              I can answer questions about the humanoid robotics book.
            </p>
            <SuggestedQuestions onSelectQuestion={handleSendMessage} />
          </div>
        ) : (
          <>
            {messages.map((message) => (
              <ChatMessage key={message.id} message={message} />
            ))}
            {isLoading && (
              <div className="flex items-center gap-3 text-gray-400 animate-fadeIn pl-12">
                <div className="w-3 h-3 bg-purple-500 rounded-full animate-bounce"></div>
                <div className="w-3 h-3 bg-blue-500 rounded-full animate-bounce" style={{ animationDelay: '0.1s' }}></div>
                <div className="w-3 h-3 bg-cyan-500 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
                <span className="font-medium">AI is thinking...</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </>
        )}
      </div>

      {/* Input Area */}
      <div className="border-t border-white/10 p-4 bg-slate-800/70 backdrop-blur-md">
        <ChatInput onSendMessage={handleSendMessage} disabled={isLoading} />
      </div>
    </div>
  );
}
