Pasos rápidos para desplegar el frontend en Vercel

1) Instala la CLI de Vercel (opcional):

```bash
npm i -g vercel
```

2) En el directorio `web/` despliega:

```bash
cd web
vercel --prod
```

3) Durante el despliegue, si te pide, configura el proyecto como un sitio estático. `vercel` leerá `vercel.json` y servirá `public/`.

4) Ajusta el frontend para conectar con el backend público (Socket.IO): en `public/app.js` o donde se inicializa `io()`, usa la URL pública del backend (p.ej. `https://mi-backend.example.com`).

5) Opcional: en el panel de Vercel configura variables de entorno y dominios personalizados.
