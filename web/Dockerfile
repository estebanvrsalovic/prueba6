FROM node:18-alpine

# Build deps for native modules (sqlite3)
RUN apk add --no-cache python3 make g++ build-base sqlite-dev

WORKDIR /app

# Install production dependencies
COPY package.json package-lock.json ./
RUN npm ci --production

# Copy app source
COPY . .

# Persist DB directory
VOLUME ["/app/data"]

ENV PORT=3000
EXPOSE 3000

CMD ["node", "server.js"]
