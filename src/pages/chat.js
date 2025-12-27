import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/ChatInterface';

function ChatPage() {
  return (
    <Layout title="Chat Interface" description="RAG Agent Chat Interface">
      <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
        <h1>RAG Agent Chat Interface</h1>
        <p>Interact with the RAG agent using the chat interface below. You can select text from book pages to provide context for your queries.</p>
        <div style={{ marginTop: '20px' }}>
          <ChatInterface />
        </div>
      </div>
    </Layout>
  );
}

export default ChatPage;