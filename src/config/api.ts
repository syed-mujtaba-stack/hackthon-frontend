// API Configuration
// For browser compatibility, we check if process is defined
const getApiUrl = () => {
    if (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) {
        return process.env.REACT_APP_API_URL;
    }
    return 'https://rag-backend-theta.vercel.app';
};

export const API_BASE_URL = getApiUrl();

export const API_ENDPOINTS = {
    RAG_ASK: `${API_BASE_URL}/rag/ask`,
    RAG_ASK_SELECTION: `${API_BASE_URL}/rag/ask-selection`,
    RAG_PERSONALIZE: `${API_BASE_URL}/rag/personalize`,
    RAG_TRANSLATE: `${API_BASE_URL}/rag/translate`,
    AUTH_SIGNUP: `${API_BASE_URL}/auth/signup`,
    AUTH_SIGNIN: `${API_BASE_URL}/auth/signin`,
};
