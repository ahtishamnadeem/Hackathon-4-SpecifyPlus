import React from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './chat.module.css';
import FloatingChatbot from '../components/FloatingChatbot';

function ChatPage() {
  return (
    <Layout title="Chat Interface" description="RAG Agent Chat Interface">
      <div className={clsx('container', styles.chatContainer)}>
        <div className={styles.chatContent}>
          <h1 className={clsx('hero__title', styles.slideInFromLeft)}>RAG Agent Chat Interface</h1>
          <p className={clsx('hero__subtitle', styles.fadeIn)}>Interact with the RAG agent using the chat interface below. You can select text from book pages to provide context for your queries.</p>
          <div className={styles.chatWrapper}>
            <FloatingChatbot />
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatPage;