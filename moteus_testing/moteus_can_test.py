import asyncio
import math
import moteus

async def main():
    c = moteus.Controller()
    await c.set_stop()  # in case there was a fault

    while True:
        # print(await c.set_position(position=math.nan, query=True))
        print(await c.query())
        print(c.id)
        return
        await asyncio.sleep(0.1)

asyncio.run(main())
