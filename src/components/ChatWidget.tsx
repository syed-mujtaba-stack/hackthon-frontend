import React, { useState, useEffect, ChangeEvent, KeyboardEvent } from 'react';
import './chat.css';
import { API_ENDPOINTS } from '../config/api';

interface Message {
    role: string;
    content: string;
}

const AIIcon = () => (
    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="24" height="24" style={{ marginRight: '8px' }}>
        <defs>
            <linearGradient id="brainGrad" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" style={{ stopColor: '#f97316', stopOpacity: 1 }} />
                <stop offset="100%" style={{ stopColor: '#dc2626', stopOpacity: 1 }} />
            </linearGradient>
            <filter id="aiGlow">
                <feGaussianBlur stdDeviation="2" result="coloredBlur" />
                <feMerge>
                    <feMergeNode in="coloredBlur" />
                    <feMergeNode in="SourceGraphic" />
                </feMerge>
            </filter>
        </defs>
        <circle cx="50" cy="50" r="35" fill="url(#brainGrad)" opacity="0.2" />
        <circle cx="50" cy="50" r="30" fill="none" stroke="url(#brainGrad)" strokeWidth="3" filter="url(#aiGlow)">
            <animate attributeName="stroke-dasharray" values="0 188; 188 0; 0 188" dur="3s" repeatCount="indefinite" />
            <animate attributeName="opacity" values="0.8; 1; 0.8" dur="2s" repeatCount="indefinite" />
        </circle>
        <g opacity="0.9">
            <circle cx="50" cy="50" r="4" fill="#fff">
                <animate attributeName="r" values="4; 5; 4" dur="1.5s" repeatCount="indefinite" />
            </circle>
            <circle cx="50" cy="30" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="0s" />
            </circle>
            <circle cx="35" cy="35" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="0.3s" />
            </circle>
            <circle cx="65" cy="35" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="0.6s" />
            </circle>
            <circle cx="50" cy="70" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="0.9s" />
            </circle>
            <circle cx="35" cy="65" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="1.2s" />
            </circle>
            <circle cx="65" cy="65" r="3" fill="#fff">
                <animate attributeName="opacity" values="0.5; 1; 0.5" dur="1.5s" repeatCount="indefinite" begin="1.5s" />
            </circle>
            <line x1="50" y1="50" x2="50" y2="30" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" />
            </line>
            <line x1="50" y1="50" x2="35" y2="35" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" begin="0.3s" />
            </line>
            <line x1="50" y1="50" x2="65" y2="35" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" begin="0.6s" />
            </line>
            <line x1="50" y1="50" x2="50" y2="70" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" begin="0.9s" />
            </line>
            <line x1="50" y1="50" x2="35" y2="65" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" begin="1.2s" />
            </line>
            <line x1="50" y1="50" x2="65" y2="65" stroke="#fff" strokeWidth="1" opacity="0.4">
                <animate attributeName="opacity" values="0.2; 0.6; 0.2" dur="1.5s" repeatCount="indefinite" begin="1.5s" />
            </line>
        </g>
        <circle r="2" fill="#fff" opacity="0.8">
            <animateMotion dur="3s" repeatCount="indefinite" path="M 50,50 m -25,0 a 25,25 0 1,0 50,0 a 25,25 0 1,0 -50,0" />
        </circle>
        <circle r="2" fill="#fff" opacity="0.6">
            <animateMotion dur="4s" repeatCount="indefinite" path="M 50,50 m -25,0 a 25,25 0 1,1 50,0 a 25,25 0 1,1 -50,0" />
        </circle>
    </svg>
);

const ChatWidget: React.FC = () => {
    const [isOpen, setIsOpen] = useState<boolean>(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState<string>('');
    const [loading, setLoading] = useState<boolean>(false);
    const [selectedText, setSelectedText] = useState<string>('');

    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection();
            const text = selection ? selection.toString() : '';
            if (text && text.length > 10) {
                setSelectedText(text);
                if (!isOpen) setIsOpen(true); // Auto-open on selection
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, [isOpen]);

    const toggleChat = () => setIsOpen(!isOpen);

    const sendMessage = async () => {
        if (!input.trim() && !selectedText) return;

        const userMsg: Message = { role: 'user', content: input };
        setMessages((prev: Message[]) => [...prev, userMsg]);
        setInput('');
        setLoading(true);

        try {
            let endpoint = API_ENDPOINTS.RAG_ASK;
            let body: any = { query: input, history: messages };

            if (selectedText) {
                endpoint = API_ENDPOINTS.RAG_ASK_SELECTION;
                body = { query: input || "Explain this selection", selected_text: selectedText };
                // Clear selection after sending
                setSelectedText('');
            }

            const response = await fetch(endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body)
            });
            const data = await response.json();
            setMessages((prev: Message[]) => [...prev, { role: 'assistant', content: data.answer }]);
        } catch (error) {
            setMessages((prev: Message[]) => [...prev, { role: 'assistant', content: "Error connecting to AI." }]);
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="chat-widget-container">
            {!isOpen && (
                <button className="chat-toggle-btn" onClick={toggleChat}>
                    <AIIcon />
                    Ask AI
                </button>
            )}
            {isOpen && (
                <div className="chat-window">
                    <div className="chat-header">
                        <h3>Physical AI Assistant</h3>
                        <button onClick={toggleChat}>X</button>
                    </div>
                    <div className="chat-messages">
                        {messages.map((msg, idx) => (
                            <div key={idx} className={`message ${msg.role}`}>
                                {msg.content}
                            </div>
                        ))}
                        {loading && <div className="message assistant">Thinking...</div>}
                    </div>

                    {selectedText && (
                        <div className="selection-preview">
                            <small>Selected: {selectedText.substring(0, 30)}...</small>
                            <button onClick={() => setSelectedText('')}>x</button>
                        </div>
                    )}

                    <div className="chat-input">
                        <input
                            value={input}
                            onChange={(e: ChangeEvent<HTMLInputElement>) => setInput(e.target.value)}
                            onKeyPress={(e: KeyboardEvent<HTMLInputElement>) => e.key === 'Enter' && sendMessage()}
                            placeholder={selectedText ? "Ask about selection..." : "Ask a question..."}
                        />
                        <button onClick={sendMessage}>Send</button>
                    </div>
                </div>
            )}
        </div>
    );
};

export default ChatWidget;
