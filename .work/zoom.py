from PIL import Image
for tag, path in (("calib", r"app\捕获\追踪夹取\标定\标定_20261030_195435.jpg"),
                  ("stop17", r"app\捕获\追踪夹取\20261030_195602\阶段_17.jpg"),
                  ("lift18", r"app\捕获\追踪夹取\20261030_195602\阶段_18.jpg")):
    im = Image.open(path).convert("RGB")
    box = (60, 200, 580, 480)
    im.crop(box).resize(((box[2]-box[0])*2, (box[3]-box[1])*2), Image.LANCZOS).save(rf".work\zoom_{tag}.jpg", quality=92)
    print(tag, "ok")
