import type { ComputedRef } from 'vue'
import { onMounted, onUnmounted, ref, watch } from 'vue'

const KONAMI_KEYS = [
  'ArrowUp',
  'ArrowUp',
  'ArrowDown',
  'ArrowDown',
  'ArrowLeft',
  'ArrowRight',
  'ArrowLeft',
  'ArrowRight',
  'b',
  'a',
] as const

function konamiKeyMatches(expected: string, e: KeyboardEvent) {
  if (expected === 'a' || expected === 'b')
    return e.key.length === 1 && e.key.toLowerCase() === expected
  return e.key === expected
}

export function useHomeKonamiEgg(isHome: ComputedRef<boolean>) {
  const eggToastVisible = ref(false)
  let eggHideTimer = 0
  let konamiProgress = 0

  function onKonamiKeydown(e: KeyboardEvent) {
    if (!isHome.value || e.altKey || e.ctrlKey || e.metaKey)
      return

    const expected = KONAMI_KEYS[konamiProgress]
    if (konamiKeyMatches(expected, e)) {
      konamiProgress++
      if (konamiProgress >= KONAMI_KEYS.length) {
        konamiProgress = 0
        eggToastVisible.value = true
        clearTimeout(eggHideTimer)
        eggHideTimer = window.setTimeout(() => {
          eggToastVisible.value = false
        }, 6500)
      }
    }
    else {
      konamiProgress = konamiKeyMatches(KONAMI_KEYS[0], e) ? 1 : 0
    }
  }

  function maybeConsoleHomeHint() {
    if (typeof window === 'undefined' || !isHome.value)
      return
    try {
      if (sessionStorage.getItem('apollo-docs-egg-console'))
        return
      sessionStorage.setItem('apollo-docs-egg-console', '1')
      console.log(
        '%c Apollo Docs %c 开发者小灶：首页可以重温「某个」经典按键组合 ✨',
        'background:#2563eb;color:#fff;padding:4px 10px;border-radius:6px;font-weight:700;',
        'color:#64748b;',
      )
    }
    catch {
      /* private mode 等 */
    }
  }

  watch(
    isHome,
    (home) => {
      if (!home)
        konamiProgress = 0
      else
        maybeConsoleHomeHint()
    },
    { immediate: true },
  )

  onMounted(() => {
    window.addEventListener('keydown', onKonamiKeydown)
  })
  onUnmounted(() => {
    window.removeEventListener('keydown', onKonamiKeydown)
    clearTimeout(eggHideTimer)
  })

  return { eggToastVisible }
}
