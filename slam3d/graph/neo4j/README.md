The cpp interface used only supports neo4jv 3.x

sudo apt install libhiredis-dev
docker run --rm --name neo4j --env=NEO4J_AUTH=none --publish=7474:7474 --publish=7687:7687 neo4j:community


Instead of using the databases in docker, you can also install netively.

To use redis-cli for debugging docker, you'll have to install redis anyways: sudo apt install redis



https://neo4j.com/docs/http-api/current/actions/begin-and-commit-a-transaction-in-one-request/


https://neo4j.com/docs/cypher-manual/current/clauses/create/



http://localhost:7474/browser/

display: `match (n) return n as node`
delete all: `match (n) detach delete n`
select all nodes from a single sensor: `match (n:Vertex)-[r:Tentative]->(m:Vertex) where r.sensor="c1.coyote3_cam_right_TOF"  return n AS node`


