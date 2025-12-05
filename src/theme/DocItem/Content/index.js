import React, { useState } from 'react';
import Content from '@theme-original/DocItem/Content';
import ReactMarkdown from 'react-markdown';
import { API_ENDPOINTS } from '../../../config/api';

export default function ContentWrapper(props) {
    const [contentMode, setContentMode] = useState('default'); // default, personalized, urdu
    const [customContent, setCustomContent] = useState(null);
    const [loading, setLoading] = useState(false);

    const handlePersonalize = async (mode) => {
        setLoading(true);
        // We need to get the text content. 
        // Since we are wrapping Content, we might not have easy access to the raw text prop unless we parse children.
        // But grabbing from DOM is a reasonable fallback for this "overlay" approach.
        // However, if we are in "default" mode, the content is rendered.
        const article = document.querySelector('article');
        const text = article ? article.innerText : "";

        if (!text) {
            alert("Could not find content to process");
            setLoading(false);
            return;
        }

        try {
            const endpoint = mode === 'urdu' ? API_ENDPOINTS.RAG_TRANSLATE : API_ENDPOINTS.RAG_PERSONALIZE;
            const body = mode === 'urdu'
                ? { text, target_language: "Urdu" }
                : { text, level: "beginner" };

            const response = await fetch(endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(body)
            });
            const data = await response.json();
            setCustomContent(data.personalized_markdown || data.translated_markdown);
            setContentMode(mode);
        } catch (error) {
            console.error(error);
            alert("Failed to process content. Backend might be offline.");
        } finally {
            setLoading(false);
        }
    };

    return (
        <>
            <div style={{ marginBottom: '20px', display: 'flex', gap: '10px', flexWrap: 'wrap' }}>
                <button
                    className={`button ${contentMode === 'default' ? 'button--secondary' : 'button--outline button--secondary'}`}
                    onClick={() => setContentMode('default')}
                    disabled={contentMode === 'default'}
                >
                    Original
                </button>
                <button
                    className="button button--primary"
                    onClick={() => handlePersonalize('personalized')}
                    disabled={loading}
                >
                    {loading && contentMode === 'personalized' ? 'Personalizing...' : '✨ Personalize'}
                </button>
                <button
                    className="button button--info"
                    onClick={() => handlePersonalize('urdu')}
                    disabled={loading}
                >
                    {loading && contentMode === 'urdu' ? 'Translating...' : '🌐 Translate to Urdu'}
                </button>
            </div>

            {contentMode === 'default' ? (
                <Content {...props} />
            ) : (
                <div className="markdown" dir={contentMode === 'urdu' ? 'rtl' : 'ltr'} style={{ padding: '20px', border: '1px solid #eee', borderRadius: '8px' }}>
                    <ReactMarkdown>{customContent}</ReactMarkdown>
                </div>
            )}
        </>
    );
}
