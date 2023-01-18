const express = require('express')
const url = require('url');
const app = express()
const port = 5000

const request = require('request')

var scope = "files.read.all"

function getVar(variable, query)
{
       var vars = {};
       var parts = query.replace(/[?&]+([^=&]+)=([^&]*)/gi, function(m,key,value) {
           vars[key] = value;
       });
       return vars[variable];
}

app.use(express.static('.'))

app.listen(port, () => console.log(`test server listening on port ${port}!`))