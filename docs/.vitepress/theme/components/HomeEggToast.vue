<script setup lang="ts">
defineProps<{
  visible: boolean
}>()
</script>

<template>
  <Teleport to="body">
    <Transition name="apollo-egg-fade">
      <div
        v-if="visible"
        class="apollo-home-egg-toast"
        role="status"
        aria-live="polite"
      >
        <span class="apollo-home-egg-icon" aria-hidden="true">🛰️</span>
        <p class="apollo-home-egg-title">
          Cyber RT 链路畅通
        </p>
        <p class="apollo-home-egg-sub">
          秘籍已识别 — 愿你的 bag 不丢帧，Planning 轨迹常绿。
        </p>
      </div>
    </Transition>
  </Teleport>
</template>

<style scoped>
.apollo-home-egg-toast {
  position: fixed;
  bottom: 1.5rem;
  left: 50%;
  z-index: 1000;
  max-width: 22rem;
  margin: 0 1rem;
  padding: 1rem 1.25rem;
  text-align: center;
  border-radius: 12px;
  background: color-mix(in srgb, var(--vp-c-bg) 92%, transparent);
  border: 1px solid var(--vp-c-divider);
  box-shadow:
    0 10px 36px rgba(30, 64, 175, 0.12),
    0 0 0 1px color-mix(in srgb, var(--vp-c-brand-1) 12%, transparent);
  backdrop-filter: blur(12px);
  transform: translateX(-50%);
}

:global(.dark) .apollo-home-egg-toast {
  box-shadow:
    0 12px 40px rgba(0, 0, 0, 0.35),
    0 0 0 1px color-mix(in srgb, var(--vp-c-brand-1) 25%, transparent);
}

.apollo-home-egg-icon {
  display: block;
  margin-bottom: 0.35rem;
  font-size: 1.5rem;
  line-height: 1;
}

.apollo-home-egg-title {
  margin: 0;
  font-size: 0.95rem;
  font-weight: 700;
  color: var(--vp-c-text-1);
}

.apollo-home-egg-sub {
  margin: 0.4rem 0 0;
  font-size: 0.8rem;
  line-height: 1.4;
  color: var(--vp-c-text-2);
}

.apollo-egg-fade-enter-active,
.apollo-egg-fade-leave-active {
  transition: opacity 0.35s ease;
}

@media (prefers-reduced-motion: reduce) {
  .apollo-egg-fade-enter-active,
  .apollo-egg-fade-leave-active {
    transition: none;
  }
}

.apollo-egg-fade-enter-from,
.apollo-egg-fade-leave-to {
  opacity: 0;
}
</style>
