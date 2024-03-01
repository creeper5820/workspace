<template>
    <div class="person">
        <h3>name: {{ name }}</h3>
        <h3>age : {{ age }}</h3>
        <button @click="change_name">change the name</button>
        <button @click="change_age">change the age</button>
        <button @click="show_tele">View the telephone number</button>
        <input>
    </div>
</template>

<script lang="ts">
export default {
    name: 'Person',
    data() {
        return {
            name: "creeper",
            age: 20,
            telephone: "10086"
        }
    },
    methods: {
        show_tele() {
            alert(this.telephone)
        },

        change_name() {
            this.age--;
        },

        change_age() {
            this.age++;
        },
    }
}
</script>

<style scoped>
.person {
    font-family: 'Courier New', Courier, monospace;
    font-size: large;

    background-color: rgb(219, 232, 227);
    box-shadow: 0 0 5px;
    border-radius: 15px;
    padding: 10px;
}

button {
    padding: 10px;
    margin: 10px;
    border-radius: 15px;
}
</style>