import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatBot.module.css';
import { API_ENDPOINTS } from '../../config/api';

interface Message {
    role: 'user' | 'assistant';
    content: string;
}

export default function ChatBot() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef<HTMLDivElement>(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    // Listen for text selection
    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection();
            const text = selection?.toString().trim();
            if (text && text.length > 0) {
                setSelectedText(text);
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async (message: string, isSelection: boolean = false) => {
        if (!message.trim()) return;

        const userMessage: Message = { role: 'user', content: message };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setIsLoading(true);

        try {
            const endpoint = isSelection ? API_ENDPOINTS.RAG_ASK_SELECTION : API_ENDPOINTS.RAG_ASK;
            const body = isSelection
                ? { query: message, selected_text: selectedText }
                : { query: message, history: messages };

            const response = await fetch(endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body),
            });

            const data = await response.json();
            const assistantMessage: Message = {
                role: 'assistant',
                content: data.answer || 'Sorry, I encountered an error.',
            };

            setMessages(prev => [...prev, assistantMessage]);
        } catch (error) {
            console.error('Error:', error);
            setMessages(prev => [
                ...prev,
                { role: 'assistant', content: 'Failed to connect to the server.' },
            ]);
        } finally {
            setIsLoading(false);
            setSelectedText('');
        }
    };

    const handleSubmit = (e: React.FormEvent) => {
        e.preventDefault();
        sendMessage(input);
    };

    const askAboutSelection = () => {
        if (selectedText) {
            sendMessage(`Explain this: "${selectedText}"`, true);
        }
    };

    return (
        <>
            {/* Floating Button */}
            <button
                className={styles.floatingButton}
                onClick={() => setIsOpen(!isOpen)}
                aria-label="Toggle chatbot"
            >
                {isOpen ? '✕' : '💬'}
            </button>

            {/* Chatbot Window */}
            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.header}>
                        <h3>Physical AI Assistant</h3>
                        <p>Ask me anything about the textbook!</p>
                    </div>

                    <div className={styles.messages}>
                        {messages.length === 0 && (
                            <div className={styles.welcomeMessage}>
                                <p>👋 Hello! I'm your Physical AI assistant.</p>
                                <p>Ask me questions about humanoid robotics, ROS 2, or any chapter content!</p>
                                {selectedText && (
                                    <button
                                        className={styles.selectionButton}
                                        onClick={askAboutSelection}
                                    >
                                        Explain selected text: "{selectedText.substring(0, 50)}..."
                                    </button>
                                )}
                            </div>
                        )}

                        {messages.map((msg, idx) => (
                            <div
                                key={idx}
                                className={`${styles.message} ${msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                                    }`}
                            >
                                <div className={styles.messageContent}>{msg.content}</div>
                            </div>
                        ))}

                        {isLoading && (
                            <div className={`${styles.message} ${styles.assistantMessage}`}>
                                <div className={styles.typing}>
                                    <span></span>
                                    <span></span>
                                    <span></span>
                                </div>
                            </div>
                        )}

                        <div ref={messagesEndRef} />
                    </div>

                    {selectedText && (
                        <div className={styles.selectionBanner}>
                            <span>Text selected: "{selectedText.substring(0, 40)}..."</span>
                            <button onClick={askAboutSelection}>Ask about this</button>
                        </div>
                    )}

                    <form className={styles.inputForm} onSubmit={handleSubmit}>
                        <input
                            type="text"
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            placeholder="Ask a question..."
                            disabled={isLoading}
                        />
                        <button type="submit" disabled={isLoading || !input.trim()}>
                            Send
                        </button>
                    </form>
                </div>
            )}
        </>
    );
}
