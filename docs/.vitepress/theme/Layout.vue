<script setup>
import DefaultTheme from 'vitepress/theme'
import HomeEggToast from './components/HomeEggToast.vue'
import { useHomeConfetti } from './composables/useHomeConfetti'
import { useHomeKonamiEgg } from './composables/useHomeKonamiEgg'
import { useIsHome } from './composables/useIsHome'
import { useMediumZoom } from './composables/useMediumZoom'

import './theme-overrides.css'

const { Layout } = DefaultTheme

const isHome = useIsHome()
useMediumZoom()
const { eggToastVisible } = useHomeKonamiEgg(isHome)
useHomeConfetti(isHome)
</script>

<template>
  <Layout />
  <HomeEggToast :visible="eggToastVisible" />
</template>
