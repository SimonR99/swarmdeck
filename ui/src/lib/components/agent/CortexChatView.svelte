<script lang="ts">
  import {
    Brain,
    Plus,
    History,
    Trash2,
    Send,
    Square,
    ChevronDown,
    ChevronRight,
    Terminal,
    FileCode,
    Search,
    Eye,
    Compass,
    CheckCircle2,
    Clock,
    AlertTriangle,
    Zap,
    Paperclip,
    X,
    Radio,
    ShieldAlert,
    Copy,
    Check,
    MessageSquare,
    Sparkles
  } from 'lucide-svelte';
  import { cortexStore, type ImageAttachment } from '$lib/stores/agent.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';

  let inputPrompt = $state('');
  let chatContainer = $state<HTMLDivElement | null>(null);
  let textareaEl = $state<HTMLTextAreaElement | null>(null);
  let fileInputEl = $state<HTMLInputElement | null>(null);
  let expandedTools = $state<Record<string, boolean>>({});
  let pendingAttachments = $state<ImageAttachment[]>([]);
  let showHistoryDrawer = $state(false);
  let copiedSnippet = $state<string | null>(null);

  // Autocomplete state
  let showMentionMenu = $state(false);
  let showSlashMenu = $state(false);
  let mentionQuery = $state('');
  let slashQuery = $state('');

  const selectedRobotId = $derived(fleet.selected[0] || null);
  const selectedRobotName = $derived(
    selectedRobotId ? robotDisplayName(selectedRobotId) : 'Fleet'
  );

  const filteredRobots = $derived(
    fleet.robots.filter((r) =>
      r.robot_id.toLowerCase().includes(mentionQuery.toLowerCase())
    )
  );

  const filteredSkills = $derived(
    cortexStore.skills.filter(
      (s) =>
        s.command.toLowerCase().includes(slashQuery.toLowerCase()) ||
        s.name.toLowerCase().includes(slashQuery.toLowerCase())
    )
  );

  function toggleTool(key: string) {
    expandedTools[key] = !expandedTools[key];
  }

  function scrollToBottom() {
    if (chatContainer) {
      chatContainer.scrollTop = chatContainer.scrollHeight;
    }
  }

  $effect(() => {
    if (cortexStore.messages.length || cortexStore.isStreaming) {
      setTimeout(scrollToBottom, 40);
    }
  });

  function handleInputChange() {
    const text = inputPrompt;
    const cursorPos = textareaEl?.selectionStart ?? text.length;
    const beforeCursor = text.slice(0, cursorPos);

    const atMatch = beforeCursor.match(/@([a-zA-Z0-9_-]*)$/);
    if (atMatch) {
      showMentionMenu = true;
      showSlashMenu = false;
      mentionQuery = atMatch[1] || '';
    } else {
      showMentionMenu = false;
    }

    const slashMatch = beforeCursor.match(/^\/([a-zA-Z0-9_-]*)$/);
    if (slashMatch) {
      showSlashMenu = true;
      showMentionMenu = false;
      slashQuery = slashMatch[1] || '';
    } else {
      showSlashMenu = false;
    }
  }

  function selectMention(robotId: string) {
    const cursorPos = textareaEl?.selectionStart ?? inputPrompt.length;
    const beforeCursor = inputPrompt.slice(0, cursorPos);
    const afterCursor = inputPrompt.slice(cursorPos);
    const updatedBefore = beforeCursor.replace(/@([a-zA-Z0-9_-]*)$/, `@${robotId} `);
    inputPrompt = updatedBefore + afterCursor;
    showMentionMenu = false;
    textareaEl?.focus();
  }

  function selectSlashCommand(cmd: string) {
    inputPrompt = `${cmd} `;
    showSlashMenu = false;
    textareaEl?.focus();
  }

  async function handleFileUpload(event: Event) {
    const target = event.target as HTMLInputElement;
    if (!target.files || target.files.length === 0) return;

    for (let i = 0; i < target.files.length; i++) {
      const file = target.files[i];
      const att = await cortexStore.uploadImage(file);
      if (att) {
        pendingAttachments.push(att);
      }
    }
    target.value = '';
  }

  function removeAttachment(idx: number) {
    pendingAttachments.splice(idx, 1);
  }

  function handleSubmit() {
    if ((!inputPrompt.trim() && pendingAttachments.length === 0) || cortexStore.isStreaming) return;
    const prompt = inputPrompt;
    const attachments = [...pendingAttachments];
    inputPrompt = '';
    pendingAttachments = [];
    showMentionMenu = false;
    showSlashMenu = false;
    cortexStore.send(prompt, attachments);
  }

  function handleKeyDown(event: KeyboardEvent) {
    if (showMentionMenu && (event.key === 'ArrowDown' || event.key === 'ArrowUp' || event.key === 'Enter')) {
      if (event.key === 'Enter' && filteredRobots.length > 0) {
        event.preventDefault();
        selectMention(filteredRobots[0].robot_id);
        return;
      }
    }

    if (showSlashMenu && event.key === 'Enter' && filteredSkills.length > 0) {
      event.preventDefault();
      selectSlashCommand(filteredSkills[0].command);
      return;
    }

    if (event.key === 'Escape') {
      if (showMentionMenu || showSlashMenu) {
        showMentionMenu = false;
        showSlashMenu = false;
        event.preventDefault();
        return;
      }
      if (showHistoryDrawer) {
        showHistoryDrawer = false;
        event.preventDefault();
        return;
      }
    }

    if (event.key === 'Enter' && !event.shiftKey) {
      event.preventDefault();
      handleSubmit();
    }
  }

  function sendQuickPrompt(promptText: string) {
    cortexStore.send(promptText);
  }

  function copyCode(code: string) {
    navigator.clipboard.writeText(code);
    copiedSnippet = code;
    setTimeout(() => {
      if (copiedSnippet === code) copiedSnippet = null;
    }, 2000);
  }

  function formatContent(text: string): string {
    if (!text) return '';
    return text
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/```([a-zA-Z0-9_-]*)\n([\s\S]*?)```/g, (_m, lang, code) => {
        return `<div class="my-2.5 rounded-xl bg-[#0f172a] border border-slate-800 p-3 font-mono text-xs overflow-x-auto text-emerald-400 shadow-sm"><div class="flex items-center justify-between text-[10px] text-slate-400 uppercase font-semibold tracking-wider mb-1.5"><span>${lang || 'code'}</span></div><pre><code>${code.trim()}</code></pre></div>`;
      })
      .replace(/`([^`]+)`/g, '<code class="bg-surface-3 border border-border text-blue-700 px-1.5 py-0.5 rounded text-[11px] font-mono font-medium">$1</code>')
      .replace(/\*\*([^*]+)\*\*/g, '<strong class="font-semibold text-fg">$1</strong>')
      .replace(/\*([^*]+)\*/g, '<em class="text-fg-muted">$1</em>')
      .replace(/^\s*-\s+(.+)$/gm, '<li class="ml-3.5 list-disc text-fg my-0.5">$1</li>')
      .replace(/@([a-zA-Z0-9_-]+)/g, '<span class="inline-flex items-center gap-0.5 rounded-md bg-blue-100/90 border border-blue-200 text-blue-800 px-1.5 py-0.2 font-mono text-[11px] font-bold">@$1</span>')
      .replace(/^### (.*$)/gim, '<h3 class="text-xs font-bold text-blue-800 mt-2.5 mb-1">$1</h3>')
      .replace(/^## (.*$)/gim, '<h2 class="text-sm font-bold text-fg mt-3 mb-1.5">$1</h2>')
      .replace(/^# (.*$)/gim, '<h1 class="text-base font-bold text-fg mt-3.5 mb-2">$1</h1>')
      .replace(/\n\n/g, '<div class="h-2"></div>')
      .replace(/\n/g, '<br/>');
  }

  function getToolSummary(tool: { name: string; params?: Record<string, any> }): string {
    if (tool.params?.CommandLine) {
      const cmd = tool.params.CommandLine.trim();
      return cmd
        .replace(/^python3?\s+scripts\//, '')
        .replace(/^python3?\s+/, '')
        .replace(/\s+--server\s+\S+/, '')
        .replace(/\s+--dangerously-skip-permissions/, '');
    }
    if (tool.params?.AbsolutePath) {
      const parts = tool.params.AbsolutePath.split('/');
      return `${tool.name} ${parts[parts.length - 1]}`;
    }
    if (tool.params?.TargetFile) {
      const parts = tool.params.TargetFile.split('/');
      return `${tool.name} ${parts[parts.length - 1]}`;
    }
    return tool.name;
  }
</script>

<div class="relative flex h-full w-full flex-col overflow-hidden bg-surface-2 text-fg font-sans">
  <!-- Top Action Strip (New Chat + History) -->
  <div class="flex items-center justify-between border-b border-border/70 px-3 py-2 bg-surface-2 shrink-0">
    <div class="flex items-center gap-1.5">
      <button
        class="flex items-center gap-1 rounded-lg bg-surface hover:bg-surface-3 border border-border px-2.5 py-1 text-[11px] font-semibold text-fg transition-all active:scale-95 shadow-2xs"
        title="Start fresh conversation"
        onclick={() => cortexStore.newChat()}
      >
        <Plus class="h-3 w-3 text-blue-600" />
        <span>New</span>
      </button>

      <button
        class="flex items-center gap-1 rounded-lg px-2 py-1 text-[11px] font-medium text-fg-muted hover:text-fg hover:bg-surface-3 transition-colors {showHistoryDrawer ? 'bg-surface text-blue-700 font-semibold shadow-2xs' : ''}"
        title="View past conversations"
        onclick={() => (showHistoryDrawer = !showHistoryDrawer)}
      >
        <History class="h-3 w-3 text-fg-dim" />
        <span>History</span>
        <span class="text-[10px] text-fg-dim font-mono">({cortexStore.threads.length})</span>
      </button>
    </div>

    <!-- Quick Shortcuts -->
    <div class="flex items-center gap-1">
      <button
        class="flex items-center gap-1 rounded-md px-2 py-0.5 text-[11px] font-medium text-fg-muted hover:text-blue-700 hover:bg-surface-3 transition-colors"
        title="Inspect camera vision"
        onclick={() => sendQuickPrompt(selectedRobotId ? `What are you seeing on @${selectedRobotId}?` : 'What are you seeing on the fleet?')}
      >
        <Eye class="h-3 w-3 text-emerald-600" />
        <span>Vision</span>
      </button>
      <button
        class="flex items-center gap-1 rounded-md px-2 py-0.5 text-[11px] font-medium text-fg-muted hover:text-blue-700 hover:bg-surface-3 transition-colors"
        title="Check robot telemetry"
        onclick={() => sendQuickPrompt('List fleet telemetry and battery')}
      >
        <Radio class="h-3 w-3 text-indigo-600" />
        <span>Status</span>
      </button>
    </div>
  </div>

  <!-- History Dropdown Overlay -->
  {#if showHistoryDrawer}
    <div class="absolute top-10 left-2 right-2 z-30 rounded-xl border border-border bg-surface p-2 shadow-xl max-h-72 overflow-y-auto space-y-1">
      <div class="flex items-center justify-between text-[10px] text-fg-dim font-semibold uppercase px-2 py-1 border-b border-border/50 pb-1.5 mb-1">
        <span>Chat History</span>
        <button
          class="text-fg-dim hover:text-fg p-0.5 rounded hover:bg-surface-3"
          onclick={() => (showHistoryDrawer = false)}
        >
          <X class="h-3.5 w-3.5" />
        </button>
      </div>

      {#each cortexStore.threads as thread (thread.id)}
        <div
          class="group flex items-center justify-between rounded-lg px-2.5 py-1.5 text-xs transition-colors {cortexStore.currentThreadId === thread.id
            ? 'bg-blue-50 text-blue-700 border border-blue-200 font-semibold'
            : 'hover:bg-surface-2 text-fg'}"
        >
          <button
            class="flex-1 flex items-center gap-2 truncate text-left"
            onclick={() => {
              cortexStore.switchThread(thread.id);
              showHistoryDrawer = false;
            }}
          >
            <MessageSquare class="h-3 w-3 text-fg-dim shrink-0 group-hover:text-blue-600" />
            <span class="truncate">{thread.title || 'New Conversation'}</span>
          </button>

          <div class="flex items-center gap-1.5 shrink-0 pl-2">
            <span class="text-[10px] text-fg-dim font-mono">{new Date(thread.updatedAt).toLocaleDateString([], { month: 'short', day: 'numeric' })}</span>
            <button
              class="opacity-0 group-hover:opacity-100 p-1 rounded hover:bg-rose-100 text-fg-dim hover:text-rose-600 transition-all"
              title="Delete conversation"
              onclick={() => cortexStore.deleteThread(thread.id)}
            >
              <Trash2 class="h-3 w-3" />
            </button>
          </div>
        </div>
      {/each}
    </div>
  {/if}

  <!-- Messages List (ChatGPT Style with high contrast) -->
  <div
    bind:this={chatContainer}
    class="flex-1 overflow-y-auto p-3 space-y-3.5 scroll-smooth bg-surface-2"
  >
    {#if cortexStore.messages.length <= 1 && cortexStore.messages[0]?.id === 'welcome'}
      <!-- Clean Minimalist Welcome Hero -->
      <div class="flex flex-col items-center justify-center text-center py-6 px-2 my-auto">
        <div class="h-11 w-11 rounded-2xl bg-gradient-to-tr from-blue-600 to-indigo-600 text-white flex items-center justify-center shadow-md shadow-blue-500/20 mb-2.5 ring-4 ring-blue-50">
          <Brain class="h-6 w-6" />
        </div>
        <h3 class="text-sm font-bold text-fg mb-0.5">How can I help you today?</h3>
        <p class="text-[11px] text-fg-muted max-w-[240px] mb-4">
          Autonomous developer & fleet operator agent.
        </p>

        <div class="grid grid-cols-1 gap-1.5 w-full max-w-[280px] text-left text-[11px]">
          <button
            class="p-2.5 rounded-xl bg-surface hover:bg-blue-50/80 border border-border hover:border-blue-300 text-fg transition-all shadow-2xs group flex items-center gap-2.5"
            onclick={() => sendQuickPrompt(selectedRobotId ? `Move @${selectedRobotId} forward 1 meter` : 'Move @aslan_0 forward 1 meter')}
          >
            <div class="h-6 w-6 rounded-lg bg-blue-50 text-blue-600 flex items-center justify-center shrink-0">
              <Zap class="h-3.5 w-3.5" />
            </div>
            <span class="truncate font-medium">Drive robot forward 1 meter</span>
          </button>

          <button
            class="p-2.5 rounded-xl bg-surface hover:bg-emerald-50/80 border border-border hover:border-emerald-300 text-fg transition-all shadow-2xs group flex items-center gap-2.5"
            onclick={() => sendQuickPrompt(selectedRobotId ? `What are you seeing on @${selectedRobotId}?` : 'What are you seeing on the fleet?')}
          >
            <div class="h-6 w-6 rounded-lg bg-emerald-50 text-emerald-600 flex items-center justify-center shrink-0">
              <Eye class="h-3.5 w-3.5" />
            </div>
            <span class="truncate font-medium">Inspect vision & camera</span>
          </button>

          <button
            class="p-2.5 rounded-xl bg-surface hover:bg-indigo-50/80 border border-border hover:border-indigo-300 text-fg transition-all shadow-2xs group flex items-center gap-2.5"
            onclick={() => sendQuickPrompt('List fleet telemetry and battery status')}
          >
            <div class="h-6 w-6 rounded-lg bg-indigo-50 text-indigo-600 flex items-center justify-center shrink-0">
              <Radio class="h-3.5 w-3.5" />
            </div>
            <span class="truncate font-medium">Fleet battery & poses</span>
          </button>
        </div>
      </div>
    {:else}
      {#each cortexStore.messages as msg (msg.id)}
        <div class="flex flex-col gap-1 {msg.role === 'user' ? 'items-end' : 'items-start'}">
          <!-- Role & Timestamp Header -->
          <div class="flex items-center gap-1.5 text-[10px] text-fg-dim px-1">
            {#if msg.role === 'user'}
              <span class="font-semibold text-fg-muted">You</span>
            {:else}
              <div class="flex items-center gap-1 text-blue-700 font-bold">
                <Brain class="h-3 w-3" />
                <span>Cortex</span>
              </div>
            {/if}
            <span>•</span>
            <span>{new Date(msg.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
          </div>

          <!-- Attachments -->
          {#if msg.attachments && msg.attachments.length > 0}
            <div class="flex flex-wrap gap-1.5 my-0.5 {msg.role === 'user' ? 'justify-end' : 'justify-start'}">
              {#each msg.attachments as att}
                <div class="rounded-xl border border-border bg-surface overflow-hidden shadow-2xs max-w-[150px]">
                  <img src={att.url} alt={att.filename} class="h-16 w-full object-cover" />
                  <div class="p-1 text-[9px] text-fg-muted truncate px-1.5 bg-surface-2">{att.filename}</div>
                </div>
              {/each}
            </div>
          {/if}

          <!-- Tool Calls (Featherweight, Unobtrusive Action Pill) -->
          {#if msg.tools && msg.tools.length > 0}
            {@const isRunning = msg.tools.some((t) => t.status === 'running')}
            {@const hasError = msg.tools.some((t) => t.status === 'error')}
            {@const groupKey = `${msg.id}_tools_group`}
            {@const isGroupExpanded = expandedTools[groupKey] ?? false}

            <div class="my-0.5 w-full">
              <button
                class="inline-flex items-center gap-1.5 rounded-full bg-surface hover:bg-surface-3 border border-border/80 px-2.5 py-0.5 text-[10px] text-fg-muted hover:text-fg transition-all active:scale-95 shadow-2xs"
                onclick={() => toggleTool(groupKey)}
              >
                {#if isRunning}
                  <Clock class="h-2.5 w-2.5 text-amber-600 animate-spin shrink-0" />
                  <span class="font-medium text-amber-700">Running action...</span>
                {:else if hasError}
                  <AlertTriangle class="h-2.5 w-2.5 text-rose-600 shrink-0" />
                  <span class="font-medium">{msg.tools.length} action{msg.tools.length > 1 ? 's' : ''} finished</span>
                {:else}
                  <Zap class="h-2.5 w-2.5 text-blue-600 shrink-0" />
                  <span class="font-medium text-fg-muted">{msg.tools.length} action{msg.tools.length > 1 ? 's' : ''}</span>
                {/if}

                <span class="text-[9px] text-fg-dim font-mono max-w-[140px] truncate">
                  {msg.tools.map((t) => getToolSummary(t)).join(', ')}
                </span>

                {#if isGroupExpanded}
                  <ChevronDown class="h-2.5 w-2.5 text-fg-dim shrink-0" />
                {:else}
                  <ChevronRight class="h-2.5 w-2.5 text-fg-dim shrink-0" />
                {/if}
              </button>

              {#if isGroupExpanded}
                <div class="mt-1 rounded-xl border border-border/70 bg-surface p-1.5 space-y-1 text-[10px] shadow-2xs">
                  {#each msg.tools as tool, idx}
                    {@const toolKey = `${msg.id}_tool_${idx}`}
                    {@const isToolExpanded = expandedTools[toolKey] ?? false}
                    <div class="rounded-lg border border-border/40 bg-surface-2/60 overflow-hidden">
                      <button
                        class="w-full flex items-center justify-between p-1 hover:bg-surface-3/60 transition-colors text-left text-fg text-[10px]"
                        onclick={() => toggleTool(toolKey)}
                      >
                        <div class="flex items-center gap-1.5 truncate">
                          <span class="font-mono text-blue-700 font-semibold">{getToolSummary(tool)}</span>
                        </div>
                        <div class="flex items-center gap-1 shrink-0">
                          <span class="text-[9px] {tool.status === 'done' ? 'text-emerald-700 font-medium' : tool.status === 'running' ? 'text-amber-700' : 'text-rose-700'}">{tool.status}</span>
                          {#if tool.output}
                            {#if isToolExpanded}
                              <ChevronDown class="h-2.5 w-2.5 text-fg-dim" />
                            {:else}
                              <ChevronRight class="h-2.5 w-2.5 text-fg-dim" />
                            {/if}
                          {/if}
                        </div>
                      </button>
                      {#if isToolExpanded && tool.output}
                        <pre class="border-t border-border/40 p-1.5 bg-[#0f172a] text-emerald-400 font-mono text-[9px] max-h-32 overflow-x-auto whitespace-pre-wrap">{tool.output}</pre>
                      {/if}
                    </div>
                  {/each}
                </div>
              {/if}
            </div>
          {/if}

          <!-- Message Body -->
          <div
            class="max-w-[95%] rounded-2xl px-3.5 py-2.5 leading-relaxed text-xs {msg.role === 'user'
              ? 'bg-blue-600 text-white shadow-2xs rounded-tr-xs ml-6'
              : 'bg-surface border border-border/90 text-fg shadow-2xs rounded-tl-xs w-full mr-2'}"
          >
            {#if msg.content}
              <div class="leading-relaxed select-text {msg.role === 'user' ? 'text-white' : 'text-fg'}">
                <!-- eslint-disable-next-line svelte/no-at-html-tags -->
                {@html formatContent(msg.content)}
              </div>
            {:else if msg.isStreaming}
              <div class="flex items-center gap-2 py-1 text-fg-muted">
                <div class="flex items-center gap-1">
                  <div class="h-1.5 w-1.5 rounded-full bg-blue-600 animate-bounce"></div>
                  <div class="h-1.5 w-1.5 rounded-full bg-blue-600 animate-bounce [animation-delay:0.2s]"></div>
                  <div class="h-1.5 w-1.5 rounded-full bg-blue-600 animate-bounce [animation-delay:0.4s]"></div>
                </div>
                <span class="text-[11px] font-medium text-fg-muted">Cortex is thinking...</span>
              </div>
            {/if}
          </div>
        </div>
      {/each}
    {/if}
  </div>

  <!-- Autocomplete Dropdowns -->
  {#if showMentionMenu && filteredRobots.length > 0}
    <div class="border-t border-border bg-surface p-1.5 shadow-lg max-h-36 overflow-y-auto space-y-0.5">
      <div class="text-[9px] text-fg-dim font-semibold uppercase px-1.5 py-0.5">Target Robot</div>
      {#each filteredRobots as r}
        <button
          class="w-full flex items-center justify-between rounded-lg px-2 py-1 hover:bg-blue-50 text-left text-xs transition-colors text-fg"
          onclick={() => selectMention(r.robot_id)}
        >
          <span class="font-mono text-blue-700 font-bold">@{r.robot_id}</span>
          <span class="text-emerald-700 text-[10px] font-medium">{Math.round((r.battery ?? 0) * 100)}%</span>
        </button>
      {/each}
    </div>
  {/if}

  {#if showSlashMenu && filteredSkills.length > 0}
    <div class="border-t border-border bg-surface p-1.5 shadow-lg max-h-44 overflow-y-auto space-y-0.5">
      <div class="text-[9px] text-fg-dim font-semibold uppercase px-1.5 py-0.5">Skills</div>
      {#each filteredSkills as skill}
        <button
          class="w-full flex items-center justify-between rounded-lg px-2 py-1 hover:bg-blue-50 text-left text-xs transition-colors text-fg"
          onclick={() => selectSlashCommand(skill.command)}
        >
          <span class="font-mono text-blue-700 font-bold">{skill.command}</span>
          <span class="text-fg-muted text-[10px] truncate max-w-[140px]">{skill.name}</span>
        </button>
      {/each}
    </div>
  {/if}

  <!-- Attachments preview in composer -->
  {#if pendingAttachments.length > 0}
    <div class="flex items-center gap-1 border-t border-border px-2 py-1 bg-surface-2 overflow-x-auto">
      {#each pendingAttachments as att, idx}
        <div class="relative group rounded-lg border border-border bg-surface p-0.5 flex items-center gap-1 shadow-2xs">
          <img src={att.url} alt={att.filename} class="h-7 w-7 object-cover rounded" />
          <span class="text-[9px] text-fg max-w-[70px] truncate px-1">{att.filename}</span>
          <button
            class="h-3.5 w-3.5 rounded-full bg-surface-3 hover:bg-rose-600 hover:text-white text-fg-dim flex items-center justify-center transition-colors"
            onclick={() => removeAttachment(idx)}
          >
            <X class="h-2 w-2" />
          </button>
        </div>
      {/each}
    </div>
  {/if}

  <!-- Floating Minimal Composer -->
  <footer class="border-t border-border/80 p-2 bg-surface-2 shrink-0">
    <div class="relative flex flex-col rounded-2xl border border-border bg-surface focus-within:border-blue-500 focus-within:ring-2 focus-within:ring-blue-500/15 transition-all p-1.5 shadow-xs">
      <textarea
        bind:this={textareaEl}
        bind:value={inputPrompt}
        oninput={handleInputChange}
        onkeydown={handleKeyDown}
        rows="2"
        placeholder="Message Cortex... (@ for robots, / for skills)"
        class="w-full resize-none bg-transparent px-2 pt-1 text-xs text-fg placeholder:text-fg-dim focus:outline-none font-sans leading-relaxed"
      ></textarea>

      <div class="flex items-center justify-between pt-1 px-1 border-t border-border/40 mt-1">
        <div class="flex items-center gap-1">
          <input
            bind:this={fileInputEl}
            type="file"
            accept="image/*"
            multiple
            class="hidden"
            onchange={handleFileUpload}
          />
          <button
            class="flex h-6 w-6 items-center justify-center rounded-lg hover:bg-surface-3 text-fg-dim hover:text-blue-600 transition-colors"
            title="Attach image"
            onclick={() => fileInputEl?.click()}
          >
            <Paperclip class="h-3.5 w-3.5" />
          </button>

          {#if selectedRobotId}
            <span class="inline-flex items-center rounded-md bg-blue-50 border border-blue-200 text-blue-700 px-1.5 py-0.2 text-[10px] font-mono font-medium">
              @{selectedRobotId}
            </span>
          {/if}
        </div>

        <div class="flex items-center gap-1">
          {#if cortexStore.isStreaming}
            <button
              class="flex h-6 items-center gap-1 rounded-full bg-rose-600 hover:bg-rose-500 px-2 text-[10px] font-semibold text-white transition-colors"
              title="Stop Generating"
              onclick={() => cortexStore.stop()}
            >
              <Square class="h-2.5 w-2.5 fill-current" />
              <span>Stop</span>
            </button>
          {:else}
            <button
              class="flex h-6 w-6 items-center justify-center rounded-full bg-blue-600 hover:bg-blue-500 text-white font-bold shadow-2xs transition-all disabled:opacity-30 disabled:hover:bg-blue-600"
              disabled={!inputPrompt.trim() && pendingAttachments.length === 0}
              title="Send (Enter)"
              onclick={handleSubmit}
            >
              <Send class="h-3 w-3" />
            </button>
          {/if}
        </div>
      </div>
    </div>
  </footer>
</div>
