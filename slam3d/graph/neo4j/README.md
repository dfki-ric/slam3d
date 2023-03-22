The cpp interface used only supports neo4jv 3.x

docker run --rm --name neo4j --env=NEO4J_AUTH=none --publish=7474:7474 --publish=7687:7687 neo4j:community



https://neo4j.com/docs/http-api/current/actions/begin-and-commit-a-transaction-in-one-request/


https://neo4j.com/docs/cypher-manual/current/clauses/create/



http://localhost:7474/browser/

display: match (n) return n as node
delete all: match (n) detach delete n

    
