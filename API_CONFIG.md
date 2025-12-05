# API Configuration

## Backend URL Setup

The frontend is configured to use the Vercel-deployed backend by default:
- **Production**: `https://rag-backend-theta.vercel.app`
- **Local Development**: `http://localhost:8000`

### Environment Variable (Optional)

You can override the backend URL by creating a `.env` file in the frontend root:

```bash
REACT_APP_API_URL=http://localhost:8000
```

### Configuration File

The API configuration is centralized in `src/config/api.ts`:

```typescript
export const API_BASE_URL = process.env.REACT_APP_API_URL || 'https://rag-backend-theta.vercel.app';
```

### Available Endpoints

All API endpoints are defined in `src/config/api.ts`:
- `RAG_ASK` - General chat queries
- `RAG_ASK_SELECTION` - Explain selected text
- `RAG_PERSONALIZE` - Content personalization
- `RAG_TRANSLATE` - Urdu translation
- `AUTH_SIGNUP` - User registration
- `AUTH_SIGNIN` - User authentication

### Components Using API

The following components make backend API calls:
- `ChatWidget.tsx` - Floating chat interface
- `ChatBot/ChatBot.tsx` - Full chatbot component
- `PersonalizeButton.tsx` - Content personalization
- `TranslateButton.tsx` - Translation feature
- `theme/DocItem/Content/index.js` - Doc page overlay features

## Deployment

When deploying to production, ensure CORS is properly configured on the backend to allow requests from your frontend domain.
