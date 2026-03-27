<script setup>
import DefaultTheme from 'vitepress/theme'
import { useMediumZoom } from './composables/useMediumZoom'

import './theme-overrides.css'

const { Layout } = DefaultTheme

useMediumZoom()
</script>

<template>
  <Layout />
</template>
