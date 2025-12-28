'use client'

interface Props {
  onSelectQuestion: (question: string) => void
}

const SUGGESTED_QUESTIONS = [
  'What is ROS 2?',
  'Explain forward kinematics',
  'How does inverse kinematics work?',
  'What is a PID controller?',
  'Explain humanoid robot dynamics',
  'How do I set up Isaac Sim?',
]

export default function SuggestedQuestions({ onSelectQuestion }: Props) {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-3 max-w-2xl mx-auto">
      {SUGGESTED_QUESTIONS.map((question, idx) => (
        <button
          key={idx}
          onClick={() => onSelectQuestion(question)}
          className="bg-white/10 hover:bg-white/20 border border-white/20 text-white px-4 py-3 rounded-lg text-sm text-left transition"
        >
          💡 {question}
        </button>
      ))}
    </div>
  )
}
