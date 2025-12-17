# ROS 2 Fundamentals Book - Deployment

## Production Build

To build the site for production:

```bash
cd frontend_book
npm run build
```

The static files will be generated in the `build` directory.

## Deployment

### Netlify

The site is configured for Netlify deployment with the `netlify.toml` file:
- Build command: `npm run build`
- Publish directory: `build`

### GitHub Pages

To deploy to GitHub Pages:

1. Set the correct `baseUrl` in `docusaurus.config.js`
2. Run: `npm run deploy`

### Other Static Hosts

Upload the contents of the `build` directory to your web server.

## Local Testing

To test the production build locally:

```bash
cd frontend_book
npx docusaurus serve
```

## Smoke Test

After deployment, verify:

- [ ] Site loads correctly
- [ ] Navigation works
- [ ] All pages are accessible
- [ ] Dark/light mode toggle works
- [ ] Search functionality works
- [ ] Links to documentation work
- [ ] Responsive design on mobile
- [ ] Performance is acceptable