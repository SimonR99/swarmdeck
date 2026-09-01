import { fleet } from './fleet.svelte';

export interface ImageAttachment {
  imageId: string;
  filename: string;
  url: string;
  path: string;
  width?: number;
  height?: number;
}

export interface AgentToolCall {
  name: string;
  params: Record<string, any>;
  output?: string;
  status: 'running' | 'done' | 'error';
}

export interface AgentMessage {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  attachments?: ImageAttachment[];
  tools?: AgentToolCall[];
  timestamp: number;
  isStreaming?: boolean;
}

export interface ChatThread {
  id: string;
  title: string;
  createdAt: number;
  updatedAt: number;
  messages: AgentMessage[];
  conversationId: string | null;
}

export interface CortexSkill {
  command: string;
  name: string;
  description: string;
  usage: string;
  examples: string[];
  category: string;
}

const STORAGE_KEY = 'swarmdeck_cortex_threads_v1';

function createDefaultMessages(): AgentMessage[] {
  return [
    {
      id: 'welcome',
      role: 'assistant',
      content:
        '🧠 **Hello! I am Cortex**, your AI Fleet Intelligence & Developer.\n\n' +
        'How can I help with the robots or codebase today?',
      timestamp: Date.now(),
    },
  ];
}

class CortexStore {
  isOpen = $state(true);
  activeTab = $state<'fleet' | 'cortex'>('fleet');
  isStreaming = $state(false);
  isUploading = $state(false);
  conversationId = $state<string | null>(null);
  error = $state<string | null>(null);
  skills = $state<CortexSkill[]>([]);

  // Threads & History
  threads = $state<ChatThread[]>([]);
  currentThreadId = $state<string>('');
  messages = $state<AgentMessage[]>([]);

  private abortController: AbortController | null = null;

  constructor() {
    this.initThreads();
    this.fetchSkills();
  }

  private initThreads() {
    let loaded: ChatThread[] = [];
    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) {
        loaded = JSON.parse(stored);
      }
    } catch {
      // Best-effort
    }

    if (loaded.length === 0) {
      const defaultId = `thread_${Date.now()}`;
      const defaultThread: ChatThread = {
        id: defaultId,
        title: 'New Conversation',
        createdAt: Date.now(),
        updatedAt: Date.now(),
        messages: createDefaultMessages(),
        conversationId: null,
      };
      loaded = [defaultThread];
    }

    this.threads = loaded;
    const initial = loaded[0];
    this.currentThreadId = initial.id;
    this.messages = initial.messages;
    this.conversationId = initial.conversationId;

    // Sync with server in background
    this.fetchServerThreads();
  }

  private async fetchServerThreads() {
    try {
      const res = await fetch('/api/agent/threads');
      if (res.ok) {
        const data = await res.json();
        // Server threads available if needed
      }
    } catch {
      // Local fallback is active
    }
  }

  private persistThreads() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.threads));
      // Async sync to server
      const current = this.threads.find((t) => t.id === this.currentThreadId);
      if (current) {
        fetch('/api/agent/threads', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(current),
        }).catch(() => {});
      }
    } catch (e) {
      console.warn('[cortex] failed saving history', e);
    }
  }

  async fetchSkills() {
    try {
      const res = await fetch('/api/agent/skills');
      if (res.ok) {
        const data = await res.json();
        this.skills = data.skills || [];
      }
    } catch {
      // Best-effort
    }
  }

  newChat() {
    const threadId = `thread_${Date.now()}`;
    const newThread: ChatThread = {
      id: threadId,
      title: 'New Conversation',
      createdAt: Date.now(),
      updatedAt: Date.now(),
      messages: createDefaultMessages(),
      conversationId: null,
    };
    this.threads.unshift(newThread);
    this.currentThreadId = threadId;
    this.messages = newThread.messages;
    this.conversationId = null;
    this.error = null;
    this.persistThreads();
  }

  switchThread(threadId: string) {
    const thread = this.threads.find((t) => t.id === threadId);
    if (!thread) return;
    this.currentThreadId = thread.id;
    this.messages = thread.messages;
    this.conversationId = thread.conversationId;
    this.error = null;
  }

  deleteThread(threadId: string) {
    this.threads = this.threads.filter((t) => t.id !== threadId);
    if (this.threads.length === 0) {
      this.newChat();
      return;
    }
    if (this.currentThreadId === threadId) {
      this.switchThread(this.threads[0].id);
    }
    this.persistThreads();
    fetch(`/api/agent/threads/${threadId}`, { method: 'DELETE' }).catch(() => {});
  }

  toggle() {
    this.isOpen = !this.isOpen;
  }

  open() {
    this.isOpen = true;
  }

  close() {
    this.isOpen = false;
  }

  openFleet() {
    this.isOpen = true;
    this.activeTab = 'fleet';
  }

  openCortex() {
    this.isOpen = true;
    this.activeTab = 'cortex';
  }

  toggleCortex() {
    if (!this.isOpen) {
      this.isOpen = true;
      this.activeTab = 'cortex';
    } else if (this.activeTab === 'cortex') {
      this.activeTab = 'fleet';
    } else {
      this.activeTab = 'cortex';
    }
  }

  clear() {
    this.newChat();
  }

  stop() {
    if (this.abortController) {
      this.abortController.abort();
      this.abortController = null;
    }
    this.isStreaming = false;
    const last = this.messages[this.messages.length - 1];
    if (last && last.role === 'assistant') {
      last.isStreaming = false;
    }
    this.persistThreads();
  }

  async uploadImage(file: File): Promise<ImageAttachment | null> {
    this.isUploading = true;
    try {
      const formData = new FormData();
      formData.append('file', file);
      const res = await fetch('/api/agent/upload', {
        method: 'POST',
        body: formData,
      });
      if (!res.ok) {
        throw new Error(`Upload failed: HTTP ${res.status}`);
      }
      const data = await res.json();
      return {
        imageId: data.image_id,
        filename: data.filename,
        url: data.url,
        path: data.path,
        width: data.width,
        height: data.height,
      };
    } catch (err: any) {
      console.error('[cortex] image upload failed', err);
      this.error = err.message || 'Image upload failed';
      return null;
    } finally {
      this.isUploading = false;
    }
  }

  async send(prompt: string, attachments: ImageAttachment[] = []) {
    const text = prompt.trim();
    if ((!text && attachments.length === 0) || this.isStreaming) return;

    this.error = null;

    // Auto-generate title if default
    const currentThread = this.threads.find((t) => t.id === this.currentThreadId);
    if (currentThread && currentThread.title === 'New Conversation' && text) {
      currentThread.title = text.length > 28 ? `${text.slice(0, 25)}...` : text;
    }

    // 1. Add user message
    this.messages.push({
      id: `user_${Date.now()}`,
      role: 'user',
      content: text,
      attachments: attachments.length > 0 ? [...attachments] : undefined,
      timestamp: Date.now(),
    });

    // 2. Add assistant placeholder message
    this.messages.push({
      id: `cortex_${Date.now()}`,
      role: 'assistant',
      content: '',
      tools: [],
      timestamp: Date.now(),
      isStreaming: true,
    });

    this.isStreaming = true;
    this.abortController = new AbortController();
    const selectedRobot = fleet.selected[0] || null;

    try {
      const response = await fetch('/api/agent/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Accept: 'text/event-stream',
        },
        body: JSON.stringify({
          prompt: text,
          conversation_id: this.conversationId,
          selected_robot: selectedRobot,
          attachments: attachments.map((a) => ({
            image_id: a.imageId,
            filename: a.filename,
            path: a.path,
          })),
        }),
        signal: this.abortController.signal,
      });

      if (!response.ok) {
        const errText = await response.text();
        throw new Error(`HTTP ${response.status}: ${errText}`);
      }

      const reader = response.body?.getReader();
      if (!reader) {
        throw new Error('No response stream returned by server');
      }

      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          const trimmed = line.trim();
          if (!trimmed || !trimmed.startsWith('data: ')) continue;
          const jsonStr = trimmed.slice(6);
          try {
            const data = JSON.parse(jsonStr);
            this.handleEvent(data);
          } catch (e) {
            console.warn('[cortex] parse SSE error', e, jsonStr);
          }
        }
      }
    } catch (err: any) {
      const last = this.messages[this.messages.length - 1];
      if (err.name === 'AbortError') {
        if (last) last.content += '\n\n*(Generation stopped)*';
      } else {
        console.error('[cortex] chat error', err);
        this.error = err.message || 'Failed to communicate with Cortex';
        if (last) last.content += `\n\n⚠️ **Error:** ${this.error}`;
      }
    } finally {
      this.isStreaming = false;
      const last = this.messages[this.messages.length - 1];
      if (last) last.isStreaming = false;
      this.abortController = null;
      this.messages = [...this.messages];

      if (currentThread) {
        currentThread.updatedAt = Date.now();
        currentThread.messages = this.messages;
        currentThread.conversationId = this.conversationId;
      }
      this.persistThreads();
    }
  }

  private handleEvent(data: Record<string, any>) {
    const lastMsg = this.messages[this.messages.length - 1];
    if (!lastMsg || lastMsg.role !== 'assistant') return;

    const type = data.type;

    if (type === 'init') {
      if (data.conversation_id) {
        this.conversationId = data.conversation_id;
      }
    } else if (type === 'token') {
      if (data.delta) {
        lastMsg.content = (lastMsg.content || '') + data.delta;
      }
    } else if (type === 'tool_call') {
      if (!lastMsg.tools) lastMsg.tools = [];
      lastMsg.tools = [
        ...lastMsg.tools,
        {
          name: data.tool,
          params: data.params || {},
          status: 'running',
        },
      ];
    } else if (type === 'tool_output') {
      if (lastMsg.tools && lastMsg.tools.length > 0) {
        const tools = [...lastMsg.tools];
        const idx = tools.findLastIndex((t) => t.name === data.tool && t.status === 'running');
        const targetIdx = idx >= 0 ? idx : tools.length - 1;
        tools[targetIdx] = {
          ...tools[targetIdx],
          output: typeof data.output === 'string' ? data.output : JSON.stringify(data.output, null, 2),
          status: 'done',
        };
        lastMsg.tools = tools;
      }
    } else if (type === 'done') {
      if (data.response && !lastMsg.content) {
        lastMsg.content = data.response;
      }
      lastMsg.isStreaming = false;
    } else if (type === 'error') {
      this.error = data.error;
      lastMsg.content = (lastMsg.content || '') + `\n\n⚠️ **Error:** ${data.error}`;
      lastMsg.isStreaming = false;
    }

    this.messages = [...this.messages];
  }
}

export const cortexStore = new CortexStore();
export const agentStore = cortexStore;
