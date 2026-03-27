import { onMounted } from 'vue'
import { useRouter } from 'vitepress'
import mediumZoom from 'medium-zoom'

export function useMediumZoom() {
  const router = useRouter()
  const setupMediumZoom = () => {
    mediumZoom('[data-zoomable]', {
      background: 'transparent',
    })
  }
  onMounted(setupMediumZoom)
  router.onAfterRouteChange = setupMediumZoom
}
