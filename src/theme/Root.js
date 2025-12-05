import React from 'react';
import ChatBot from '@site/src/components/ChatBot/ChatBot';

export default function Root({ children }) {
    return (
        <>
            {children}
            <ChatBot />
        </>
    );
}
