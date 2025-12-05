import React, { useState } from 'react';
import ReactMarkdown from 'react-markdown';
import { API_ENDPOINTS } from '../config/api';

const PersonalizeButton = () => {
    const [loading, setLoading] = useState(false);
    const [content, setContent] = useState<string | null>(null);

    const handlePersonalize = async () => {
        setLoading(true);
        const article = document.querySelector('article');
        if (!article) return;
        const text = article.innerText;

        try {
            const response = await fetch(API_ENDPOINTS.RAG_PERSONALIZE, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text, level: 'beginner' })
            });
            const data = await response.json();
            setContent(data.personalized_markdown);
        } catch (e) {
            console.error(e);
            alert("Failed to personalize content");
        } finally {
            setLoading(false);
        }
    };

    if (content) {
        return (
            <div className="personalized-content" style={{ padding: '20px', backgroundColor: '#f0f8ff', borderRadius: '8px' }}>
                <div className="banner" style={{ marginBottom: '10px', fontWeight: 'bold' }}>
                    Personalized Content (Beginner)
                    <button onClick={() => setContent(null)} style={{ marginLeft: '10px' }}>Reset</button>
                </div>
                <ReactMarkdown>{content}</ReactMarkdown>
            </div>
        );
    }

    return (
        <button onClick={handlePersonalize} disabled={loading} className="button button--primary margin-bottom--md">
            {loading ? 'Personalizing...' : '✨ Personalize this Chapter'}
        </button>
    );
};

export default PersonalizeButton;
