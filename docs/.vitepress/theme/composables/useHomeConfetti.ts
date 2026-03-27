import type { ComputedRef } from 'vue'
import { onMounted, onUnmounted, watch } from 'vue'

async function fireHomeConfetti(e: MouseEvent) {
  if (typeof window === 'undefined')
    return
  if (window.matchMedia?.('(prefers-reduced-motion: reduce)').matches)
    return
  const { default: confetti } = await import('canvas-confetti')
  const x = e.clientX / window.innerWidth
  const y = e.clientY / window.innerHeight
  confetti({
    particleCount: 80,
    spread: 62,
    startVelocity: 32,
    gravity: 1.05,
    decay: 0.92,
    scalar: 0.9,
    origin: { x, y },
    disableForReducedMotion: true,
    colors: ['#2563eb', '#1d4ed8', '#0ea5e9', '#38bdf8', '#60a5fa', '#7dd3fc'],
  })
}

export function useHomeConfetti(isHome: ComputedRef<boolean>) {
  function onHomeDocumentClick(e: MouseEvent) {
    if (!isHome.value)
      return
    const el = e.target
    if (!(el instanceof Element))
      return
    if (!el.closest('.VPHome'))
      return
    if (el.closest('a, button, input, textarea, select, summary, [role="button"]'))
      return

    void fireHomeConfetti(e)
  }

  function syncHomeConfettiClick() {
    if (typeof document === 'undefined')
      return
    document.removeEventListener('click', onHomeDocumentClick, true)
    if (isHome.value)
      document.addEventListener('click', onHomeDocumentClick, true)
  }

  watch(isHome, () => {
    syncHomeConfettiClick()
  }, { immediate: true })

  onMounted(() => {
    syncHomeConfettiClick()
  })
  onUnmounted(() => {
    if (typeof document !== 'undefined')
      document.removeEventListener('click', onHomeDocumentClick, true)
  })
}
