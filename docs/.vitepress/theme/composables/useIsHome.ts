import { computed } from 'vue'
import { useData } from 'vitepress'

export function useIsHome() {
  const { page } = useData()
  return computed(() => page.value.relativePath === 'index.md')
}
