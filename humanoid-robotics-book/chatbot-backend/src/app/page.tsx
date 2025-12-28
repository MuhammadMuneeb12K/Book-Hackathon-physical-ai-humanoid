// src/app/page.tsx

import ChatInterface from '@/components/chat/ChatInterface';

/**
 * The main page of the application.
 *
 * This is a Next.js App Router SERVER COMPONENT.
 * It renders static UI (header, cards, layout) on the server
 * and delegates all interactive chat behavior to ChatInterface,
 * which is a CLIENT COMPONENT.
 *
 * This clean separation prevents hydration mismatch errors.
 */
export default function Home() {
  return (
    <main className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white">
      <div className="container mx-auto px-4 py-8 max-w-4xl">
        {/* Header Section */}
        <div className="text-center mb-8">
          <div className="inline-block mb-4">
            {/* Static emoji – safe for SSR */}
            <div className="text-6xl">🤖</div>
          </div>

          <h1 className="text-5xl md:text-6xl font-bold bg-gradient-to-r from-purple-400 via-pink-400 to-blue-400 bg-clip-text text-transparent mb-4">
            Humanoid Robotics Assistant
          </h1>

          <p className="text-xl text-gray-300 max-w-2xl mx-auto">
            Your AI-powered guide to ROS 2, Kinematics, Dynamics, and Control
          </p>
        </div>

        {/* Static Info Cards */}
        <div className="grid md:grid-cols-3 gap-4 mb-8">
          <div className="bg-gradient-to-br from-purple-500/20 to-blue-500/20 backdrop-blur-lg rounded-2xl p-6 border border-white/10 shadow-lg">
            <div className="text-4xl mb-3">📚</div>
            <h3 className="text-white font-bold text-lg mb-2">4+ Modules</h3>
            <p className="text-gray-300 text-sm">Complete robotics curriculum</p>
          </div>

          <div className="bg-gradient-to-br from-blue-500/20 to-cyan-500/20 backdrop-blur-lg rounded-2xl p-6 border border-white/10 shadow-lg">
            <div className="text-4xl mb-3">⚡</div>
            <h3 className="text-white font-bold text-lg mb-2">Instant RAG</h3>
            <p className="text-gray-300 text-sm">Powered by Gemini & OpenAI</p>
          </div>

          <div className="bg-gradient-to-br from-cyan-500/20 to-purple-500/20 backdrop-blur-lg rounded-2xl p-6 border border-white/10 shadow-lg">
            <div className="text-4xl mb-3">🎯</div>
            <h3 className="text-white font-bold text-lg mb-2">Citations</h3>
            <p className="text-gray-300 text-sm">Every answer sourced</p>
          </div>
        </div>

        {/* Client-only interactive chat */}
        <ChatInterface />

        {/* Footer */}
        <div className="mt-8 text-center text-gray-500 text-sm">
          <p>💡 Tip: Ask specific questions about ROS 2, kinematics, dynamics, or control systems.</p>
        </div>
      </div>
    </main>
  );
}
