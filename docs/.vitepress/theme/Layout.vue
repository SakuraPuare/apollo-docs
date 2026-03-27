<script setup>
import DefaultTheme from 'vitepress/theme'
import { onMounted } from 'vue'
import { useRouter } from 'vitepress'
import mediumZoom from 'medium-zoom'

import './theme-overrides.css'

const { Layout } = DefaultTheme
const router = useRouter()

const setupMediumZoom = () => {
  mediumZoom('[data-zoomable]', {
    background: 'transparent',
  })
}

onMounted(setupMediumZoom)
router.onAfterRouteChange = setupMediumZoom
</script>

<template>
  <Layout />
</template>
