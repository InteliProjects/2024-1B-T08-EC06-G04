import tinydb


# Função que lê um registro do banco de dados pelo ID
async def read(id):
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        result = db.search(User.id == id)
        db.close()
        if result:
            return result[0]
        else:
            return {"error": "No record found with the specified id"}
    except Exception as e:
        return {"error": str(e)}


# Funcão que lê todos os registros do banco de dados
async def read_all():
    try:
        db = tinydb.TinyDB("../database/db.json")
        User = tinydb.Query()
        results = db.search(User.id.exists())
        db.close()
        if results:
            return results
        else:
            return {"error": "No records found"}
    except Exception as e:
        return {"error": str(e)}
