'use client'

import { useState, KeyboardEvent } from 'react'

interface Props {
  onSendMessage: (message: string) => void
  disabled?: boolean
}

export default function ChatInput({ onSendMessage, disabled }: Props) {
  const [input, setInput] = useState('')

  const handleSend = () => {
    const message = input.trim()
    if (!message) {
      console.warn('No message provided.') // log empty message attempts
      return
    }

    if (disabled) return

    onSendMessage(message)
    setInput('')
  }

  const handleKeyPress = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault()
      handleSend()
    }
  }

  return (
    <div className="flex gap-3 items-end">
      <textarea
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyPress}
        placeholder="Ask about ROS 2, kinematics, dynamics..."
        disabled={disabled}
        rows={1}
        className="flex-1 bg-white/10 border border-white/20 rounded-xl px-4 py-3 text-white placeholder-gray-400 focus:outline-none focus:ring-2 focus:ring-purple-500 resize-none disabled:opacity-50"
        style={{ minHeight: '50px', maxHeight: '120px' }}
      />
      <button
        onClick={handleSend}
        disabled={disabled || !input.trim()}
        className="bg-gradient-to-r from-purple-600 to-blue-600 text-white px-6 py-3 rounded-xl font-semibold hover:opacity-90 transition disabled:opacity-50 disabled:cursor-not-allowed"
      >
        Send
      </button>
    </div>
  )
}
