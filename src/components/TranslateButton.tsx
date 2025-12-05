import React, { useState } from 'react';
import ReactMarkdown from 'react-markdown';
import { API_ENDPOINTS } from '../config/api';

const TranslateButton = () => {
    const [loading, setLoading] = useState(false);
    const [content, setContent] = useState<string | null>(null);

    const handleTranslate = async () => {
        setLoading(true);
        const article = document.querySelector('article');
        if (!article) return;
        const text = article.innerText;

        try {
            const response = await fetch(API_ENDPOINTS.RAG_TRANSLATE, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ text, target_language: 'Urdu' })
            });
            const data = await response.json();
            setContent(data.translated_markdown);
        } catch (e) {
            console.error(e);
            alert("Failed to translate content");
        } finally {
            setLoading(false);
        }
    };

    if (content) {
        return (
            <div className="translated-content" dir="rtl" style={{ padding: '20px', backgroundColor: '#fff0f5', borderRadius: '8px' }}>
                <div className="banner" style={{ marginBottom: '10px', fontWeight: 'bold', direction: 'ltr' }}>
                    Urdu Translation
                    <button onClick={() => setContent(null)} style={{ marginLeft: '10px' }}>Reset</button>
                </div>
                <ReactMarkdown>{content}</ReactMarkdown>
            </div>
        );
    }

    return (
        <button onClick={handleTranslate} disabled={loading} className="button button--secondary margin-bottom--md margin-left--sm">
            {loading ? 'Translating...' : '🌐 Translate to Urdu'}
        </button>
    );
};

export default TranslateButton;
