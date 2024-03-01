<template>
    <div class="app">
        <h1>
            Hello World!
        </h1>
    </div>

    <Person />
</template>

<script lang="ts">
import Person from "./components/Person.vue"
export default {
    name: 'App',
    components: {
        Person
    }
}
</script>

<style scoped>
.app {
    font-size: medium;
    font-family: 'Courier New', Courier, monospace;

    color: aliceblue;
    background-color: rgb(169, 91, 91);

    box-shadow: 20 20 20px;
    border-radius: 10px;

    padding: 20px;
    margin: 20px;
}
</style>